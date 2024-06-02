#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiGratuitous.h>
#include <LittleFS.h>
#include <uri/UriRegex.h>

#include <PicoUtils.h>
#include <PicoMQTT.h>
#include <PicoSyslog.h>
#include <ArduinoJson.h>

PicoUtils::PinInput button(D1, true);
PicoUtils::ResetButton reset_button(button);

PicoUtils::PinOutput wifi_led(D4, true);
PicoUtils::WiFiControlSmartConfig wifi_control(wifi_led);

PicoUtils::ShiftRegister<1> shift_register(
    D6,  // data pin
    D5,  // clock pin
    D0,  // latch pin
(uint8_t[1]) { 0b11111111, }  // inverted outputs
);

PicoSyslog::Logger syslog("schalter");

PicoMQTT::Client mqtt;
PicoUtils::Stopwatch last_command_stopwatch;

const String board_id(ESP.getChipId(), HEX);
const char CONFIG_FILE[] PROGMEM = "/config.json";
String hostname;
String password;

PicoUtils::RestfulServer<ESP8266WebServer> server(80);

class SchalterOutput: public PicoUtils::ShiftRegisterOutput {
    public:
        SchalterOutput(PicoUtils::ShiftRegisterInterface & shift_register, unsigned int idx, const String & name)
            : PicoUtils::ShiftRegisterOutput(shift_register, idx), name(name), announced_state(false) {

            auto handler = [this](String payload) {
                set(payload == "ON");
                last_command_stopwatch.reset();
            };

            mqtt.subscribe("schalter/" + name + "/set", handler);
        }

        void announce_state() {
            announced_state = get();
            const char * state = announced_state ? "ON" : "OFF";
            mqtt.publish("schalter/" + name, state);
            announcement_stopwatch.reset();
        }

        void tick() {
            if (announced_state != get()) {
                syslog.printf("Relay #%i '%s' is now %s.\n", output_idx, name.c_str(), get() ? "ON" : "OFF");
                announce_state();
            } else if (announcement_stopwatch.elapsed_millis() >= 30 * 1000) {
                announce_state();
            }
        }

        const String name;

    protected:
        PicoUtils::Stopwatch announcement_stopwatch;
        bool announced_state;
};

std::vector<SchalterOutput *> outputs;

void setup() {
    wifi_led.init();
    wifi_led.set(true);

    shift_register.init();

    Serial.begin(115200);
    Serial.println(F(
                       "\n"
                       "   Schalter " __DATE__ " " __TIME__ "\n"
                       "\n\n"
                   ));

    reset_button.init();

    LittleFS.begin();

    {
        PicoUtils::JsonConfigFile<JsonDocument> config(LittleFS, FPSTR(CONFIG_FILE));
        hostname = config["hostname"] | "schalter";
        password = config["password"] | "";
        syslog.server = config["syslog"] | "";

        mqtt.host = config["mqtt"]["server"] | "calor.local";
        mqtt.port = config["mqtt"]["port"] | 1883;
        mqtt.username = config["mqtt"]["username"] | "";
        mqtt.password = config["mqtt"]["password"] | "";

        for (JsonVariant value : config["outputs"].as<JsonArray>()) {
            const unsigned int idx = outputs.size();
            const String name = value | String(idx);
            outputs.push_back(new SchalterOutput(shift_register, idx, name));
        }
    }

    WiFi.hostname(hostname);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    experimental::ESP8266WiFiGratuitous::stationKeepAliveSetIntervalMs(1000);
    wifi_control.init(button);

    wifi_control.get_connectivity_level = [] {
        return (last_command_stopwatch.elapsed_millis() <= 2 * 60 * 1000) ? 2 : 1;
    };

    mqtt.connected_callback = [] {
        syslog.println("MQTT connected, sending state updates...");
        for (auto output : outputs) { output->announce_state(); }
    };
    mqtt.begin();

    server.on("/status", HTTP_GET, [] {
        JsonDocument json;

        json["board_id"] = board_id;

        for (unsigned int idx = 0; idx < outputs.size(); ++idx) {
            SchalterOutput * output = outputs[idx];
            json["relays"][idx] = output->get();
            json["names"][idx] = output->name;
        }

        server.sendJson(json);
    });

    server.on(UriRegex("/output/([0-9]+)$"), HTTP_GET, [] {
        const auto idx = server.decodedPathArg(0).toInt();

        if (idx < 0 || idx >= (int) outputs.size()) {
            server.send(404);
        } else {
            server.send(200, "text/plain", outputs[idx]->get() ? "on" : "off");
        }
    });

    server.on(UriRegex("/output/([0-9]+)/(on|off)$"), HTTP_POST, [] {
        if (password.length() && !server.authenticate("schalter", password.c_str())) {
            return server.requestAuthentication();
        }

        const auto idx = server.decodedPathArg(0).toInt();

        if (idx < 0 || idx >= (int) outputs.size()) {
            server.send(404);
            return;
        }

        outputs[idx]->set(server.decodedPathArg(1) == "on");
        server.send(200);
    });

    server.begin();

    ArduinoOTA.setHostname(hostname.c_str());
    if (password.length()) { ArduinoOTA.setPassword(password.c_str()); }
    ArduinoOTA.begin();
}

void restart_on_connectivity_loss() {
    if (last_command_stopwatch.elapsed_millis() >= 5 * 60 * 1000) {
        ESP.restart();
    }
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    mqtt.loop();
    for (auto output : outputs) { output->tick(); }
    wifi_control.tick();
    restart_on_connectivity_loss();
}
