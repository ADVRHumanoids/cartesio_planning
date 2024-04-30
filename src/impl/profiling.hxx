#ifndef PROFILING_HXX
#define PROFILING_HXX

#include <chrono>
#include <map>
#include <vector>
#include <math.h>

#define TIC(name) auto tic_##name = std::chrono::high_resolution_clock::now()
#define TOC(name) std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - tic_##name).count()

struct SimpleStats
{

    std::vector<double> samples;
    double min;
    double max;
    double avg;
    double avg2;

    SimpleStats()
    {
        samples.reserve(1e6);
        reset();
    }

    void reset()
    {
        samples.clear();
        max = std::numeric_limits<double>::lowest();
        min = std::numeric_limits<double>::max();
        avg = 0;
        avg2 = 0;
    }

    void add(double value)
    {
        int n = samples.size();

        samples.push_back(value);
        min = std::min(min, value);
        max = std::max(max, value);
        avg = (avg*n + value)/(n+1);
        avg2 = (avg2*n + value*value)/(n+1);
    }

};

struct ProfilingData
{
    SimpleStats
        sample_q,
        update_model_state,
        state_valid,
        collision_check,
        collision_near,
        collision_near_ok,
        state_validity_check;

    std::vector<std::pair<std::string, SimpleStats*>> entries = {
        {"sample_q                 ", &sample_q},
        {"update_model_state       ", &update_model_state},
        {"state_valid              ", &state_valid},
        {"state_validity_check     ", &state_validity_check},
        {" └──collision_check      ", &collision_check},
        {"    └──collision_near    ", &collision_near},
        {"    └──collision_near_ok ", &collision_near_ok}
    };

    void reset()
    {
        for(auto [n, ss] : entries)
        {
            ss->reset();
        }
    }

    void print(std::ostream& os, double total_time)
    {
        for(auto [n, ss] : entries)
        {
            os << n << " : " << int(100*ss->avg*ss->samples.size()/total_time) << "%,  N = " << ss->samples.size() << ", " << ss->min << " < " << ss->avg << " +- " << std::sqrt(ss->avg2 - ss->avg*ss->avg) << " < " << ss->max << "\n";
        }
    }

    static ProfilingData& instance()
    {
        static ProfilingData pd;
        return pd;
    }
};

struct TikToken
{
    TikToken(SimpleStats& ss):
        _ss(ss)
    {
        t0 = std::chrono::high_resolution_clock::now();
    }

    ~TikToken()
    {
        _ss.add(std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - t0).count());
    }

    std::chrono::high_resolution_clock::time_point t0;

    SimpleStats& _ss;
};

#define TIKTOK(name) TikToken __tk_##name(ProfilingData::instance().name)

#endif // PROFILING_HXX
