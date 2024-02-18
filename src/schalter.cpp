#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiGratuitous.h>
#include <LittleFS.h>
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

std::vector<PicoUtils::Watch<bool>*> watches;
PicoMQ picomq;

PicoUtils::RestfulServer<ESP8266WebServer> server(80);

void announce_state(unsigned int idx) {
    const char * state = outputs[idx]->get() ? "ON" : "OFF";
    Serial.printf("Publishing state of schalter %i: %s\n", idx, state);
    picomq.publish("schalter/" + board_id + "/" + String(idx), state);
}

PicoUtils::PeriodicRun announce_state_proc(15, [] {
    if (outputs.empty())
        return;
    static unsigned int idx = 0;
    announce_state(idx);
    idx = (idx + 1) % outputs.size();
});

class PingPong {
    public:
        PingPong(): pings(0), pongs(0) {
            picomq.subscribe("calor/pong", [this](const char * payload) {
                if (get_ping_pong_payload() == payload) {
                    pong_stopwatch.reset();
                    ++pongs;
                }
            });
        }

        void tick() {
            if (ping_stopwatch.elapsed() >= 30) {
                ++pings;
                picomq.publish("calor/ping", get_ping_pong_payload());
                ping_stopwatch.reset();
            }
        }

        unsigned long pong_elapsed_millis() const {
            return pong_stopwatch.elapsed_millis();
        }

        unsigned long get_pings() const { return pings; }
        unsigned long get_pongs() const { return pongs; }

    protected:
        unsigned long pings, pongs;
        PicoUtils::Stopwatch ping_stopwatch, pong_stopwatch;

        String get_ping_pong_payload() const {
            return hostname + " " + String(pings);
        }
};

PingPong ping_pong;

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

        auto pp = json["pingpong"];
        pp["pings"] = ping_pong.get_pings();
        pp["pongs"] = ping_pong.get_pongs();
        pp["elapsed"] = ping_pong.pong_elapsed_millis() / 1000;

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
    }

    setup_wifi();

    for (unsigned int idx = 0; idx < output_count; ++idx) {
        outputs.push_back(new PicoUtils::ShiftRegisterOutput(shift_register, idx));
        picomq.subscribe("schalter/" + board_id + "/" + String(idx) + "/set", [idx](String payload) {
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

    if (!outputs.empty())
        announce_state_proc.interval_millis /= outputs.size();
}

void update_status_led() {
    if (WiFi.status() == WL_CONNECTED) {
        if (ping_pong.pong_elapsed_millis() <= 2 * 60 * 1000) {
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
    const unsigned long timeout = ping_pong.get_pongs() ? 3 * 60 * 1000 : 15 * 60 * 1000;

    if (ping_pong.pong_elapsed_millis() >= timeout) {
        ESP.restart();
    }
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    picomq.loop();
    ping_pong.tick();
    update_status_led();
    for (auto watch : watches) { watch->tick(); }
    announce_state_proc.tick();
    restart_on_connectivity_loss();
}
