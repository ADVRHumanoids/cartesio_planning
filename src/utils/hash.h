#include <vector>
#include <string.h>
#include <algorithm>
#include <openssl/sha.h>

class posturalHash 
{
public:
    size_t operator()(std::vector<double> v) const
    {
        std::vector<int> v_int;
        for (int i = 0; i < v.size(); i++)
        {
            v[i] *= 10000;
            v_int.push_back(v[i]);
        }
        int sum = std::accumulate(v_int.begin(), v_int.end(), 0);
        
        size_t result = (size_t)sum;
        return result;
    };
 
};