#include "Parser.h"
#include "Debug.h"


int Parser::parse(String string, Command *command) {
    // Check for empty string and split it into <command> <channel/pin> <arg>
    if (!string.endsWith("\r\n\r\n")) {
        return 1;
    }
    if (split(string, command) != 0) {
        return 1;
    }
    // Validate command and return
    return validate(*command);
}

int Parser::split(String string, Command *command) {
    // String should be in the format <command_code> <channel> [<arg>]\r\n\r\n
    // Trim string and check if empty
    string.trim();
    if (string.length() <= 0) {
        return 1;
    }
    int firstSpace = string.indexOf(' ');
    if (firstSpace != -1) {
        command->type = string.substring(0, firstSpace);
        int nextSpace = string.indexOf(' ', firstSpace + 1);
        if (nextSpace != -1) {
            command->channel = string.substring(firstSpace + 1, nextSpace).toInt();
            command->arg = string.substring(nextSpace + 1).toFloat();
        } else {
            command->channel = string.substring(firstSpace).toInt();
        }
    } else {
        command->type = string.substring(0);
    }
    return 0;
}

int Parser::validate(Command command) {
    // Only channel must be provided for the following commands
    if (command.type == ENABLE_SYMBOL || 
        command.type == DISABLE_SYMBOL) {
        return (!hasChannel(command) || hasArg(command));
    }

    // Channel and argument must be provided for the following commands
    if (command.type == STEPS_SYMBOL || 
        command.type == TIME_SYMBOL || 
        command.type == PIN_SYMBOL) {
        return (!hasChannel(command) || !hasArg(command));
    }

    // Neither channel nor argument is required for the following commands
    if (command.type == STATUS_SYMBOL || 
        command.type == RESTART_SYMBOL) {
        return (hasChannel(command) || hasArg(command));
    }

    // Channel is provided but arg is optional
    if (command.type == START_SYMBOL) {
        return (hasArg(command));
    }

    // Unknown command type received
    return 1;
}

int Parser::hasChannel(Command command) {
    return command.channel != UNDEFINED;
}

int Parser::hasArg(Command command) {
    return command.arg != UNDEFINED;
}
