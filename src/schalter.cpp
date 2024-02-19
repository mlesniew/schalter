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
PicoUtils::Blink led_blinker(wifi_led, 0, 91);

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
        SchalterOutput(PicoUtils::ShiftRegisterInterface & shift_register, unsigned int idx)
            : PicoUtils::ShiftRegisterOutput(shift_register, idx), announced_state(false) {

            mqtt.subscribe("schalter/" + board_id + "/" + String(idx) + "/set", [this](String payload) {
                set(payload == "ON");
                last_command_stopwatch.reset();
            });
        }

        void announce_state() {
            mqtt.publish("schalter/" + board_id + "/" + String(output_idx), get() ? "ON" : "OFF");
            announcement_stopwatch.reset();
            announced_state = get();
        }

        void tick() {
            if (announced_state != get()) {
                syslog.printf("Relay %i is now %s.\n", output_idx, get() ? "ON" : "OFF");
                announce_state();
            } else if (announcement_stopwatch.elapsed_millis() >= 30 * 1000) {
                announce_state();
            }
        }

    protected:
        PicoUtils::BinaryOutput * output;

        PicoUtils::Stopwatch announcement_stopwatch;
        bool announced_state;
};

std::vector<SchalterOutput *> outputs;

void setup_wifi() {
    PicoUtils::BackgroundBlinker bb(led_blinker);

    WiFi.hostname(hostname);
    WiFi.setAutoReconnect(true);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    experimental::ESP8266WiFiGratuitous::stationKeepAliveSetIntervalMs(1000);

    Serial.println(F("Press button now to enter SmartConfig."));
    led_blinker.set_pattern(1);
    const PicoUtils::Stopwatch stopwatch;
    bool smart_config = false;
    {
        while (!smart_config && (stopwatch.elapsed_millis() < 5 * 1000)) {
            smart_config = button;
            delay(100);
        }
    }

    if (smart_config) {
        led_blinker.set_pattern(0b100100100 << 9);

        Serial.println(F("Entering SmartConfig mode."));
        WiFi.beginSmartConfig();
        while (!WiFi.smartConfigDone() && (stopwatch.elapsed_millis() < 5 * 60 * 1000)) {
            delay(100);
        }

        if (WiFi.smartConfigDone()) {
            Serial.println(F("SmartConfig success."));
        } else {
            Serial.println(F("SmartConfig failed.  Reboot."));
            ESP.reset();
        }
    } else {
        WiFi.softAPdisconnect(true);
        WiFi.begin();
    }

    server.on("/status", HTTP_GET, [] {
        JsonDocument json;

        json["board_id"] = board_id;

        for (unsigned int idx = 0; idx < outputs.size(); ++idx) {
            json["relays"][idx] = outputs[idx]->get();
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

    led_blinker.set_pattern(0b10);
}

void setup() {
    shift_register.init();
    wifi_led.init();
    wifi_led.set(true);

    Serial.begin(115200);
    Serial.println(F(
                       "\n"
                       "   Schalter " __DATE__ " " __TIME__ "\n"
                       "\n\n"
                   ));

    delay(3000);

    reset_button.init();

    LittleFS.begin();

    unsigned int output_count;
    {
        PicoUtils::JsonConfigFile<JsonDocument> config(LittleFS, FPSTR(CONFIG_FILE));
        output_count = config["outputs"] | 8;
        hostname = config["hostname"] | "schalter";
        password = config["password"] | "schalter";
        syslog.server = config["syslog"] | "";

        mqtt.host = config["mqtt"]["server"] | "calor.local";
        mqtt.port = config["mqtt"]["port"] | 1883;
        mqtt.username = config["mqtt"]["username"] | "";
        mqtt.password = config["mqtt"]["password"] | "";
    }

    setup_wifi();

    for (unsigned int idx = 0; idx < output_count; ++idx) {
        outputs.push_back(new SchalterOutput(shift_register, idx));
    }

    mqtt.connected_callback = [] {
        syslog.println("MQTT connected, sending state updates...");
        for (auto output : outputs) { output->announce_state(); }
    };
    mqtt.begin();

    ArduinoOTA.setHostname(hostname.c_str());
    if (password.length()) { ArduinoOTA.setPassword(password.c_str()); }
    ArduinoOTA.begin();
}

void update_status_led() {
    if (WiFi.status() == WL_CONNECTED) {
        if (last_command_stopwatch.elapsed_millis() <= 2 * 60 * 1000) {
            led_blinker.set_pattern(uint64_t(0b101) << 60);
        } else {
            led_blinker.set_pattern(uint64_t(0b1) << 60);
        }
    } else {
        led_blinker.set_pattern(0b1100);
    }
    led_blinker.tick();
};

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
    update_status_led();
    restart_on_connectivity_loss();
}
