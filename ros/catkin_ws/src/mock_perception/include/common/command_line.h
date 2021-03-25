//
// Created by dyq on 2020/10/27.
//

#ifndef ACE_COMMON_COMMAND_LINE_H_
#define ACE_COMMON_COMMAND_LINE_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>

namespace ace {
namespace common {

// Base class for option/switch
class Option
{
public:
    // Constructor with short name/long name
    Option(char d, std::string name) : c(d), used(false), long_name(name) {}

    virtual ~Option() = default;
    virtual bool Check(int& argc, char* argv[]) = 0; //  Option found at argv[0]?
    virtual Option* Clone() const = 0;               //  Copy

public:
    char c;                   // Option letter (eg 's' for option -s)
    bool used;                // Does the command line use that option?
    std::string long_name;     // Optional long name (eg "switch" for --switch)
};

// Option on/off is called a switch
class OptionSwitch : public Option
{
public:
    // Constructor with short name/long name (optional)
    OptionSwitch(char c, std::string name = "") : Option(c,name) {}

    // Find switch in argv[0]
    bool Check(int& argc, char* argv[]) override
    {
        if(std::string("-") +c == argv[0] || (!long_name.empty() && std::string("--") + long_name==argv[0]))
        {
            used = true;
            std::rotate(argv, argv + 1, argv + argc);
            argc -= 1;
            return true;
        }
        else if(std::string(argv[0]).find(std::string("-")+c)==0)
        {
            used = true; // Handle multiple switches in single option
            std::rotate(argv[0] + 1, argv[0] + 2, argv[0] + std::string(argv[0]).size() + 1);
            return true;
        }
        return false;
    }

    // Copy
    Option* Clone() const override
    {
        return new OptionSwitch(c, long_name);
    }
};

// Option with an argument of type T, which must be readable by operator>>
template <class T>
class OptionField : public Option
{
public:
    // Constructor. The result with be stored in variable @field.
    OptionField(char c, T& field, std::string name = "")
    : Option(c, name)
    , field_(field) {}

    // Find option in argv[0] and argument in argv[1]. Throw an exception
    // (type std::string) if the argument cannot be read.
    bool Check(int& argc, char* argv[]) override
    {
        std::string param; int arg=0;
        if(std::string("-") + c == argv[0] || (!long_name.empty() && std::string("--") + long_name == argv[0]))
        {
            if(argc<=1)
            {
                throw std::string("Option ") +argv[0]+" requires argument";
            }
            param = argv[1];
            arg = 2;
        }
        else if(std::string(argv[0]).find(std::string("-")+c)==0)
        {
            param = argv[0]  +2;
            arg=1;
        }
        else if(!long_name.empty() && std::string(argv[0]).find(std::string("--") + long_name+'=')==0)
        {
            size_t size = (std::string("--") + long_name+'=').size();
            param = std::string(argv[0]).substr(size); arg=1;
        }
        if(arg>0)
        {
            if(! ReadParam(param)) {
                throw std::string("Unable to interpret ") + param+" as argument of "+argv[0];
            }

            used = true;
            std::rotate(argv, argv+arg, argv+argc);
            argc -= arg;
            return true;
        }

        return false;
    }

    // Decode the string as template type T
    bool ReadParam(const std::string& param)
    {
        std::stringstream str(param); char unused;
        return !((str >> field_).fail() || !(str>>unused).fail());
    }

    // Copy
    Option* Clone() const override
    {
        return new OptionField<T>(c, field_, long_name);
    }

private:
    T& field_; // Reference to variable where to store the value
};

// Template specialization to be able to take parameter including space.
// Generic method would do >>field_ (stops at space) and signal unused chars.
template <>
inline bool OptionField<std::string>::ReadParam(const std::string& param)
{
    field_ = param;
    return true;
}

// New switch option
OptionSwitch MakeSwitch(char c, std::string name="")
{
    return OptionSwitch(c, name);
}

// New option with argument.
template <class T>
OptionField<T> MakeOption(char c, T& field, std::string name="")
{
    return OptionField<T>(c, field, name);
}

// Command line parsing
class CommandLine
{
public:
    // Destructor
    ~CommandLine()
    {
        std::vector<Option*>::iterator it = opts_.begin();
        for(; it != opts_.end(); ++it) {
            delete *it;
        }
    }

    /// Add an option
    void Add(const Option& opt)
    {
        opts_.push_back( opt.Clone() );
    }

    // Parse of command line acting as a filter. All options are virtually
    // removed from the command line.
    void Process(int& argc, char* argv[])
    {
        std::vector<Option*>::iterator it=opts_.begin();
        for(; it != opts_.end(); ++it) {
            (*it)->used = false;
        }

        for(int i=1; i<argc; )
        {
            if(std::string("--")==argv[i])
            {
                // "--" means stop option parsing
                std::rotate(argv+i, argv+i+1, argv+argc);
                -- argc;
                break;
            }

            bool found=false; // Find option
            for(it=opts_.begin(); it != opts_.end(); ++it)
            {
                int n = argc-i;
                found = (*it)->Check(n, argv+i);
                if(found) {
                    argc = n+i;
                    break;
                }
            }

            if(! found)
            {
                // A negative number is not an option
                if(std::string(argv[i]).size()>1 && argv[i][0] == '-')
                {
                    std::istringstream str(argv[i]);
                    float v;
                    if(! (str>>v).eof())
                    {
                        throw std::string("Unrecognized option ") + argv[i];
                    }
                }
                ++i;
            }
        }
    }

    // Was the option used in last parsing?
    bool Used(char c) const
    {
        std::vector<Option*>::const_iterator it=opts_.begin();
        for(; it != opts_.end(); ++it)
        {
            if((*it)->c == c) return (*it)->used;
        }

        assert(false); // Called with non-existent option, probably a bug
        return false;
    }

private:
    std::vector<Option*> opts_;
};

}  // namespace common
}  // namespace ace


#endif
