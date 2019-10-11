#include "validity_predicate_aggregate.h"
#include <iostream>

using namespace XBot::Cartesian::Planning;


bool ValidityPredicateAggregate::remove(const std::string& id)
{
    if(_functions.count(id) == 0)
    {
        return false;
    }

    _functions.erase(id);

    _expected_values.erase(id);

    return true;
}

bool ValidityPredicateAggregate::checkAll(std::vector<std::string> * failed_predicates)
{
    if(failed_predicates)
    {
        failed_predicates->clear();
    }

    if(_functions.empty())
    {
        return true;
    }

    bool success = true;

    for(auto & fn : _functions)
    {
        if(_expected_values.at(fn.first) != fn.second())
        {
            success = false;

            if(failed_predicates) // we are requested to fill the failed predicates vector
            {
                failed_predicates->push_back(fn.first);
            }
            else // no vector to be filled, just return false
            {
                return false;
            }
        }
    }

    return success;
}

bool ValidityPredicateAggregate::check(const std::string& id)
{
    if(_functions.count(id) == 0)
    {
        throw std::runtime_error("given id %s does not exists");
    }

    if(_functions[id]() != _expected_values[id])
    {
        return false;
    }
    return true;
}
