#ifndef __MY_GRID_DECOMPOSITION_H_
#define __MY_GRID_DECOMPOSITION_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>


class MyGridDecomposition : public ompl::control::GridDecomposition
{
public:
    MyGridDecomposition(const int length, const ompl::base::RealVectorBounds& bounds);

    
    void project(const ompl::base::State* state,
                 std::vector<double>& coord) const override;

    
    void sampleFullState(const ompl::base::StateSamplerPtr& sampler, 
                         const std::vector<double>& coord,
                         ompl::base::State *state) const override;
};

#endif