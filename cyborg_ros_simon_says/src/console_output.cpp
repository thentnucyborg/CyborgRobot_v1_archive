#include "ntnu_cyborg_simon_says/console_output.h"

#include <iostream>

ntnu_cyborg_simon_says::ConsoleOutput::ConsoleOutput()
{
}

void ntnu_cyborg_simon_says::ConsoleOutput::say(const std::string& text)
{
    std::cout << "ConsoleOutput -> " << text << std::endl;
}

