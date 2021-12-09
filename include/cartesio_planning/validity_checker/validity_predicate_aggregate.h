#ifndef VALIDITY_PREDICATE_AGGR_H
#define VALIDITY_PREDICATE_AGGR_H

#include <map>
#include <functional>
#include <vector>

namespace XBot { namespace Cartesian { namespace Planning {

/**
 * @brief The ValidityPredicateAggregate class contains all the predicate (functions) used for the validity
 * check
 */
class ValidityPredicateAggregate
{

public:

    /**
     * @brief add a new function to evaluate for validity check
     * @param fn a function which return a bool
     * @param id an id associated to the function
     * @param expected_value the validty check pass if func() == expected_value
     * @return false if a function with the same id already exists
     */
    template<typename Function>
    bool add(Function && fn, const std::string& id, const bool expected_value = true)
    {
        if(_functions.count(id) == 1)
            return false;
        _functions[id] = std::forward<Function>(fn);
        _expected_values[id] = expected_value;
        return true;
    }

    /**
     * @brief remove a fucntion to evaluate
     * @param id of the function to remove
     * @return false if the id does not exist
     */
    bool remove(const std::string& id);

    /**
     * @brief checkAll checks all the function for validity check
     * @param failed_predicates vectors of the fucntions which failed
     * @return false if at least one function failed
     */
    bool checkAll(std::vector<std::string>* failed_predicates = nullptr);

    /**
     * @brief check a single function
     * @param id of the fucntion to check
     * @return true if suceed, false otherwise
     * NOTE: throw error if id does not exists
     */
    bool check(const std::string& id);

private:

    std::map<std::string, std::function<bool()>> _functions;

    std::map<std::string, bool> _expected_values;
};

} } }

#endif
