#include "Parser.h"


void Parser::parse(String string, Command *command) {
    String chr = string.substring(0, 1);
    String channel = string.substring(1, 2);

    if (chr == "E") {
        command->type = COMMAND_ENABLE_MOTOR;
        command->channel = channel.toInt();

    } else if (chr == "D") {
        command->type = COMMAND_DISABLE_MOTOR;
        command->channel = channel.toInt();

    } else if (chr == "S") {
        command->type = COMMAND_STEPS_INPUT;
        command->channel = channel.toInt();
        command->arg = string.substring(2).toInt();

    } else if (chr == "T") {
        command->type = COMMAND_TIME_INPUT;
        command->channel = channel.toInt();
        command->arg = string.substring(2).toInt();

    } else if (chr == "G") {
        if (channel == "\n") {
            command->type = COMMAND_START_ALL;
        } else {
            command->type = COMMAND_START_MOTOR;
            command->channel = channel.toInt();
        }
    } else if (chr == "?") {
        command->type = COMMAND_STATUS;
    } else if (chr == "R") {
        command->type = COMMAND_RESTART
    }
}
