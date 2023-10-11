#include <Arduino.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>

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
    (uint8_t[1]) { 0b00001111, }  // inverted outputs
);

PicoSyslog::Logger syslog("schalter");

const std::vector<PicoUtils::BinaryOutput *> outputs = {
    new PicoUtils::ShiftRegisterOutput(shift_register, 0),
    new PicoUtils::ShiftRegisterOutput(shift_register, 1),
    new PicoUtils::ShiftRegisterOutput(shift_register, 2),
    new PicoUtils::ShiftRegisterOutput(shift_register, 3),
};

const String board_id(ESP.getChipId(), HEX);
const char CONFIG_FILE[] PROGMEM = "/config.json";
String hostname;

std::vector<PicoUtils::Watch<bool>*> watches;
PicoMQTT::Client mqtt;

void announce_state() {
    for (auto * watch: watches)
        watch->fire();
}

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
    {
        PicoUtils::JsonConfigFile<StaticJsonDocument<1024>> config(LittleFS, FPSTR(CONFIG_FILE));
        mqtt.host = config["mqtt"]["server"] | "";
        mqtt.port = config["mqtt"]["port"] | 1883;
        mqtt.username = config["mqtt"]["username"] | "";
        mqtt.password = config["mqtt"]["password"] | "";
        hostname = config["mqtt"]["hostname"] | "schalter";
        syslog.server = config["syslog"] | "";
    }

    setup_wifi();

    for (unsigned int idx = 0; idx < outputs.size(); ++idx) {
        mqtt.subscribe("schalter/" + board_id + "/" + String(idx) + "/set", [idx](String payload) {
            if (payload == "ON") {
                outputs[idx]->set(true);
            } else if (payload == "OFF") {
                outputs[idx]->set(false);
            }
        });

        watches.push_back(new PicoUtils::Watch<bool>(
                    [idx] { return outputs[idx]->get(); },
                    [idx] (bool value) {
                        const char * state = value ? "ON": "OFF";
                        syslog.printf("Relay %i is now %s.\n", idx, state);
                        mqtt.publish("schalter/" + board_id + "/" + String(idx), state, 0, true);
                    }));
    }

    mqtt.begin();
    mqtt.connected_callback = announce_state;

    ArduinoOTA.setHostname(hostname.c_str());
    ArduinoOTA.begin();
}

PicoUtils::PeriodicRun announce_state_proc(15, announce_state);

void update_status_led() {
    if (WiFi.status() == WL_CONNECTED) {
        if (mqtt.connected()) {
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
    mqtt.loop();
    update_status_led();
    announce_state_proc.tick();
}
