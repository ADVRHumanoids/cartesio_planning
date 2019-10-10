#ifndef __PARSE_YAML_UTILS_H__
#define __PARSE_YAML_UTILS_H__

#include <iostream>
#include <vector>
#include <string>
#include <list>

/* Macro for option parsing */
#define PLANNER_PARSE_OPTION(name, type) \
    if(opt[#name]) \
{ \
    type value = opt[#name].as<type>(); \
    std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
    planner->set##name(value); \
    } \
    else { \
    std::cout << "No option '" #name "' specified" << std::endl; \
    } \
    /* End macro for option parsing */

/* Macro for option parsing */
#define YAML_PARSE_OPTION(yaml, name, type, default_value) \
    type name = default_value; \
    if(yaml[#name]) \
{ \
    type value = yaml[#name].as<type>(); \
    std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
    name = value; \
    } \
    else { \
    std::cout << "No option '" #name "' specified, using default" << std::endl; \
    } \
    /* End macro for option parsing */

namespace std
{

template <typename T>
inline std::ostream& operator<<(std::ostream& os, std::vector<T> v)
{
    os << "\n";
    for(const auto& elem : v)
    {
        os << " - " << elem << "\n";
    }

    return os;
}

inline std::ostream& operator<<(std::ostream& os, std::list<std::string> v)
{
    os << "\n";
    for(const auto& elem : v)
    {
        os << " - " << elem << "\n";
    }

    return os;
}

}

#endif
