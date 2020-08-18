#ifndef PARSER_H
#define PARSER

#include "WString.h"

#define COMMAND_INVALID -1
#define COMMAND_ENABLE_MOTOR 0
#define COMMAND_DISABLE_MOTOR 1
#define COMMAND_STEPS_INPUT 2
#define COMMAND_TIME_INPUT 3
#define COMMAND_START_MOTOR 4
#define COMMAND_START_ALL 5
#define COMMAND_STATUS 6
#define COMMAND_RESTART 7


struct Command
{
    int type = COMMAND_INVALID;
    int channel = COMMAND_INVALID;
    float arg = COMMAND_INVALID;
};


class Parser {
    public:
    void parse(String string, Command *command);
};

#endif
