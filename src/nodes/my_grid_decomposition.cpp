#include "my_grid_decomposition.h"

MyGridDecomposition::MyGridDecomposition(const int length, const ompl::base::RealVectorBounds& bounds)
: GridDecomposition(length, 3, bounds)
{
}

void MyGridDecomposition::project(const ompl::base::State* state,
                                  std::vector<double>& coord) const
{
    coord.resize(2);
    coord[0] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    coord[1] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
}

void MyGridDecomposition::sampleFullState(const ompl::base::StateSamplerPtr& sampler,
                                          const std::vector<double>& coord,
                                          ompl::base::State* state) const
{
    sampler->sampleUniform(state);
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = coord[0];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = coord[1];
}
