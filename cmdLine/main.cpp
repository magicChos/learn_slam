#include <iostream>
#include "cmdLine.h"

using namespace std;

int main(int argc , char *argv[])
{
    cout << "Hello World!" << endl;

    std::string name = "";
    std::string location = "";

    CmdLine cmd;
    cmd.add( make_option('n' , name , "input_name") );
    cmd.add( make_option('l' , location , "input_location") );

    try {
        if (argc == 1)
            throw std::string("Invalid command line parameter.");
        cmd.process(argc , argv);
    } catch (const std::string &s)
    {
        std::cerr << "Usage: " << argv[0] << "\n"
        << "[-n|--input_name]\n"
        << "[-l|--location]\n"
        << std::endl;
    }

    std::cout << "input_name = " << name << std::endl;
    std::cout << "input_location = " << location << std::endl;
    return 0;
}
