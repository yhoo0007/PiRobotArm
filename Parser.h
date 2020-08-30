#ifndef PARSER_H
#define PARSER

#include "WString.h"

#define COMMAND_INVALID "I"
#define UNDEFINED -1

#define ENABLE_SYMBOL "E"

#define DISABLE_SYMBOL "D"

#define STEPS_SYMBOL "S"

#define TIME_SYMBOL "T"

#define START_SYMBOL "G"

#define STATUS_SYMBOL "?"

#define RESTART_SYMBOL "R"

#define PIN_SYMBOL "P"


struct Command
{
    String type = "";
    int channel = UNDEFINED;
    float arg = UNDEFINED;
};


class Parser {
    public:
    int parse(String string, Command *command);

    private:
    int split(String string, Command *command);
    int validate(Command command);
    int hasChannel(Command command);
    int hasArg(Command command);
};

#endif
