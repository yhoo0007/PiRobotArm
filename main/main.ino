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
        commandParser.parse(recv, &command);
        switch (command.type) {
        case COMMAND_INVALID:
            DPRINTLN("Invalid command received!");
            SERIAL_COM.println("1\r\n");
            break;
        case COMMAND_ENABLE_MOTOR:
            DPRINTLN("Enabling channel: " + String(command.channel));
            if (motorController.enable(command.channel) != 0) {
                DPRINTLN("Error enabling channel");
                SERIAL_COM.println("1\r\n");
            } else {
                DPRINTLN("Channel " + String(command.channel) + " enabled");
                SERIAL_COM.println("0\r\n");
            }
            break;
        case COMMAND_DISABLE_MOTOR:
            DPRINTLN("Disabling channel: " + String(command.channel));
            if (motorController.disable(command.channel) != 0) {
                DPRINTLN("Error disabling channel");
                SERIAL_COM.println("1\r\n");
            } else {
                DPRINTLN("Channel " + String(command.channel) + " disabled");
                SERIAL_COM.println("0\r\n");
            }
            break;
        case COMMAND_STEPS_INPUT:
            DPRINTLN("Setting steps: " + String(command.channel) + " to: " + String(command.arg));
            if (motorController.setSteps(command.channel, command.arg) != 0) {
                DPRINTLN("Error setting steps");
                SERIAL_COM.println("1\r\n");
            } else {
                DPRINTLN("Channel " + String(command.channel) + " set to steps " + String(command.arg));
                SERIAL_COM.println("0\r\n");
            }
            break;
        case COMMAND_TIME_INPUT:
            DPRINTLN("Setting time: " + String(command.channel) + " to: " + String(command.arg));
            if (motorController.setTime(command.channel, command.arg) != 0) {
                DPRINTLN("Error setting time");
                SERIAL_COM.println("1\r\n");
            } else {
                DPRINTLN("Channel " + String(command.channel) + " set to time " + String(command.arg));
                SERIAL_COM.println("0\r\n");
            }
            break;
        case COMMAND_START_MOTOR:
            DPRINTLN("Moving: " + String(command.channel));
            motorController.move(command.channel);
            while (motorController.running()) yield();
            SERIAL_COM.println("0\r\n");
            break;
        case COMMAND_START_ALL:
            DPRINTLN("Moving all");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motorController.move(i);
            }
            while (motorController.running()) yield();
            SERIAL_COM.println("0\r\n");
            break;
        case COMMAND_STATUS:
            DPRINTLN("Status query");
            if (motorController.running()) {
                SERIAL_COM.println("1\r\n");
            } else {
                SERIAL_COM.println("0\r\n");
            }
            break;
        case COMMAND_RESTART:
            DPRINTLN("Restarting");
            ESP.restart();
            break;
        default:
            DPRINTLN("Unknown command!");
            break;
        }
    }
}


String readSerial() {
    char chr;
    String buf = "";
    do {
        chr = SERIAL_COM.read();
        buf += String(chr);
    } while (SERIAL_COM.available() && chr != '\n');
    return buf;
}
