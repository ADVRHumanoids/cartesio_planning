#ifndef VALIDITY_PREDICATE_AGGR_H
#define VALIDITY_PREDICATE_AGGR_H

#include <map>
#include <functional>

namespace XBot { namespace Cartesian { namespace Planning {


class ValidityPredicateAggregate
{
    public:
        template<typename Function>
        bool add(Function && fn, const std::string& id, const bool expected_value = true)
        {
            if(_functions.count(id) == 1)
                return false;
            _functions[id] = std::forward<Function>(fn);
            _expected_values[id] = expected_value;
            return true;
        }

        bool remove(const std::string& id);

        bool checkAll();
    private:
        std::map<std::string, std::function<bool()>> _functions;
        std::map<std::string, bool> _expected_values;
};

}
}
}

#endif
