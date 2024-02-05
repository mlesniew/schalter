#include <Arduino.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ESP8266WebServer.h>
#include <uri/UriRegex.h>

#include <PicoUtils.h>
#include <PicoMQ.h>
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

std::vector<PicoUtils::BinaryOutput *> outputs;

const String board_id(ESP.getChipId(), HEX);
const char CONFIG_FILE[] PROGMEM = "/config.json";
String hostname;
String password;

PicoUtils::Stopwatch last_command_stopwatch;
std::vector<PicoUtils::Watch<bool>*> watches;
PicoMQ picomq;

PicoUtils::RestfulServer<ESP8266WebServer> server(80);

void announce_state(unsigned int idx) {
    const char * state = outputs[idx]->get() ? "ON" : "OFF";
    Serial.printf("Publishing state of schalter %i: %s\n", idx, state);
    picomq.publish("schalter/" + board_id + "/" + String(idx), state);
}

PicoUtils::PeriodicRun announce_state_proc(15, [] {
    static unsigned int idx = 0;
    announce_state(idx);
    idx = (idx + 1) % outputs.size();
});

void setup_wifi() {
    WiFi.hostname(hostname);
    WiFi.setAutoReconnect(true);

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
        StaticJsonDocument<1024> json;

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

    reset_button.init();

    LittleFS.begin();

    unsigned int output_count;
    {
        PicoUtils::JsonConfigFile<StaticJsonDocument<1024>> config(LittleFS, FPSTR(CONFIG_FILE));
        output_count = config["outputs"] | 8;
        hostname = config["hostname"] | "schalter";
        password = config["password"] | "schalter";
        syslog.server = config["syslog"] | "";
    }

    setup_wifi();

    for (unsigned int idx = 0; idx < output_count; ++idx) {
        outputs.push_back(new PicoUtils::ShiftRegisterOutput(shift_register, idx));
        picomq.subscribe("schalter/" + board_id + "/" + String(idx) + "/set", [idx](String payload) {
            last_command_stopwatch.reset();
            if (payload == "ON") {
                outputs[idx]->set(true);
            } else if (payload == "OFF") {
                outputs[idx]->set(false);
            }
        });

        watches.push_back(new PicoUtils::Watch<bool>(
                              [idx] { return outputs[idx]->get(); },
        [idx](bool value) {
            syslog.printf("Relay %i is now %s.\n", idx, value ? "ON" : "OFF");
            announce_state(idx);
        }));
    }

    picomq.begin();

    ArduinoOTA.setHostname(hostname.c_str());
    if (password.length()) { ArduinoOTA.setPassword(password.c_str()); }
    ArduinoOTA.begin();

    announce_state_proc.interval_millis /= outputs.size();
}

void update_status_led() {
    if (WiFi.status() == WL_CONNECTED) {
        if (last_command_stopwatch.elapsed() <= 60) {
            led_blinker.set_pattern(uint64_t(0b101) << 60);
        } else {
            led_blinker.set_pattern(uint64_t(0b1) << 60);
        }
    } else {
        led_blinker.set_pattern(0b1100);
    }
    led_blinker.tick();
};

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    picomq.loop();
    update_status_led();
    for (auto watch : watches) { watch->tick(); }
    announce_state_proc.tick();
}
