#include "serial_cmd.h"
#include "motor.h"
#include <Arduino.h>

static String cmd;

void serial_command() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (cmd == "st") stop_all();
            else if (cmd == "fo") forward(150);
            cmd = "";
        } else {
            cmd += c;
        }
    }
}
