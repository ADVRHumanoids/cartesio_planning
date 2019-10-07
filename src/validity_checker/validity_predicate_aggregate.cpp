#include "validity_predicate_aggregate.h"
#include <iostream>

using namespace XBot::Cartesian::Planning;


bool ValidityPredicateAggregate::remove(const std::string& id)
{
    if(_functions.count(id) == 0)
        return false;
    _functions.erase(id);
    _expected_values.erase(id);
    return true;
}

bool ValidityPredicateAggregate::checkAll()
{
    if(!_functions.empty())
    {
       for(auto & fn : _functions)
       {
           if(_expected_values[fn.first])
           {
               if(!fn.second())
                   return false;
           }
           else
           {
               if(fn.second())
                   return false;
           }

       }

       return true;
    }
    return false;
}
