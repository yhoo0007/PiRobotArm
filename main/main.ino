#include "Parser.h"
#include "Motor.h"
#include "MotorController.h"

// #define DEBUG
#ifdef DEBUG
#define DEBUG_SERIAL Serial
#define DPRINT_BEGIN(baud) DEBUG_SERIAL.begin(baud)
#define DPRINT(string) DEBUG_SERIAL.print(string)
#define DPRINTLN(string) DEBUG_SERIAL.println(string)
#define DPRINTLN_ARR(arr, n) for (int _iterator_ = 0; _iterator_ < n; _iterator_++) DEBUG_SERIAL.println(arr[_iterator_])
#else
#define DPRINT_BEGIN(baud)
#define DPRINT(string)
#define DPRINTLN(string)
#define DPRINTLN_ARR(arr, n)
#endif

// Serial command parameters
#define COM_RX 14
#define COM_TX 27
#define SERIAL_COM Serial
#define sendDoneSignal() SERIAL_COM.println("0\r\n\r\n")
#define sendErrorSignal() SERIAL_COM.println("1\r\n\r\n")
Parser commandParser;

#define FOR_PICKER

#ifdef FOR_PICKER
#define NUM_MOTORS 1
#else
#define NUM_MOTORS 3
#endif

void setup() {
    DPRINT_BEGIN(115200);
    // SERIAL_COM.begin(115200, SERIAL_8N1, COM_RX, COM_TX);
    SERIAL_COM.begin(115200);
    DPRINTLN("Running setup");
    
    // Prepare array of up to 3 motor configs to be passed into the motor controller.
    MotorConfig configs[NUM_MOTORS];
    #ifndef FOR_PICKER
    configs[0].stepPin = 22;
    configs[0].dirPin = 21;
    configs[0].timer = timerBegin(0, 80, true);
    configs[0].isr = &motorIsr0;

    configs[1].stepPin = 4;
    configs[1].dirPin = 16;
    configs[1].timer = timerBegin(1, 80, true);
    configs[1].isr = &motorIsr1;

    configs[2].stepPin = 5;
    configs[2].dirPin = 17;
    configs[2].timer = timerBegin(2, 80, true);
    configs[2].isr = &motorIsr2;
    #else
    // Picker motor config
    configs[0].stepPin = 16;
    configs[0].dirPin = 17;
    configs[0].enaPin = 4;
    configs[0].timer = timerBegin(0, 80, true);
    configs[0].isr = &motorIsr0;
    #endif

    if (motorController.init(NUM_MOTORS, configs) != 0) {
        DPRINTLN("Error setting up motor controller");
        delay(2000);
        return;
    }
    DPRINTLN("Setup complete");
}


void loop() {
    serveSerial();
}


void serveSerial() {
    if (SERIAL_COM.available()) {
        String recv = readSerial();
        Command command;
        DPRINT(recv);
        int validity = commandParser.parse(recv, &command);
        bool error = false;
        if (validity != 0) {
            error = true;
        } else if (command.type == ENABLE_SYMBOL) {
            DPRINTLN("Enabling channel: " + String(command.channel));
            if (motorController.enable(command.channel) != 0) {
                DPRINTLN("Error enabling channel");
                error = true;
            }
        } else if (command.type == DISABLE_SYMBOL) {
            DPRINTLN("Disabling channel: " + String(command.channel));
            if (motorController.disable(command.channel) != 0) {
                DPRINTLN("Error disabling channel");
                error = true;
            }
        } else if (command.type == STEPS_SYMBOL) {
            DPRINTLN("Setting steps: " + String(command.channel) + " to: " + String(command.arg));
            if (motorController.setSteps(command.channel, command.arg) != 0) {
                DPRINTLN("Error setting steps");
                error = true;
            }
        } else if (command.type == TIME_SYMBOL) {
            DPRINTLN("Setting time :" + String(command.channel) + " to: " + String(command.arg));
            if (motorController.setTime(command.channel, command.arg) != 0) {
                DPRINTLN("Error setting time");
                error = true;
            }
        } else if (command.type == START_SYMBOL) {
            if (command.channel == -1) {
                DPRINTLN("Moving all");
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motorController.move(i);
                }
            } else {
                DPRINTLN("Moving: " + String(command.channel));
                motorController.move(command.channel);
            }
            while (motorController.running()) yield();
        } else if (command.type == STATUS_SYMBOL) {
            DPRINTLN("Status query");
            if (motorController.running()) {
                error = true;
            }
        } else if (command.type == RESTART_SYMBOL) {
            DPRINTLN("Restarting");
            sendDoneSignal();
            ESP.restart();
        } else if (command.type == PIN_SYMBOL) {
            DPRINTLN("Setting pin: " + String(command.channel) + " to: " + String(command.arg));
            digitalWrite(command.channel, command.arg);
        } else {
            DPRINTLN("Unknown command received");
        }
        if (error) {
            sendErrorSignal();
        } else {
            sendDoneSignal();
        }
    }
}


String readSerial() {
    String buf = "";
    while (SERIAL_COM.available() && !buf.endsWith("\r\n\r\n")) {
        buf += String(char(SERIAL_COM.read()));
    }
    return buf;
}
