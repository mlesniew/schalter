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
        enum class State {
            deactivating = 0b00,
            inactive = 0b01,
            activating = 0b10,
            active = 0b11,
        };

        static const char * to_cstr(const State & state) {
            switch (state) {
                case State::activating:
                    return "TON";
                case State::deactivating:
                    return "TOFF";
                case State::active:
                    return "ON";
                case State::inactive:
                default:
                    return "OFF";
            }
        }

        SchalterOutput(PicoUtils::ShiftRegisterInterface & shift_register, unsigned int idx, const String & name,
                       unsigned long delay_ms)
            : PicoUtils::ShiftRegisterOutput(shift_register, idx), name(name), delay_ms(delay_ms),
              announced_state(State::inactive) {

            auto handler = [this](String payload) {
                set(payload == "ON");
                last_command_stopwatch.reset();
            };

            mqtt.subscribe("schalter/" + name + "/set", handler);
        }

        virtual void set(const bool new_value) override {
            if (new_value != get()) {
                activation_stopwatch.reset();
            }
            PicoUtils::ShiftRegisterOutput::set(new_value);
        }

        State get_state() const {
            const unsigned int v =
                (get() ? 0b10 : 0b00) |
                (activation_stopwatch.elapsed_millis() >= delay_ms ? 0b01 : 0b00);
            return static_cast<State>(v);
        }

        void announce_state() {
            announced_state = get_state();
            mqtt.publish("schalter/" + name, to_cstr(announced_state));
            announcement_stopwatch.reset();
        }

        void tick() {
            const auto state = get_state();
            if (announced_state != state) {
                syslog.printf("Relay #%i '%s' is now %s.\n", output_idx, name.c_str(), to_cstr(state));
                announce_state();
            } else if (announcement_stopwatch.elapsed_millis() >= 30 * 1000) {
                announce_state();
            }
        }

        const String name;
        const unsigned long delay_ms;

    protected:
        PicoUtils::Stopwatch announcement_stopwatch;
        PicoUtils::Stopwatch activation_stopwatch;
        State announced_state;
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
            const String name = value["name"] | (board_id + "-" + String(idx));
            const unsigned long delay_ms = (value["delay"] | 0.0) * 1000;
            outputs.push_back(new SchalterOutput(shift_register, idx, name, delay_ms));
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
            json["states"][idx] = SchalterOutput::to_cstr(output->get_state());
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
