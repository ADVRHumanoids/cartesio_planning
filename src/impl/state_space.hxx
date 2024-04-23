#ifndef STATE_SPACE_HXX
#define STATE_SPACE_HXX

#include <cartesio_planning/state_space.h>

#include <ompl/base/StateSpace.h>

#include <ompl/base/StateSpaceTypes.h>

namespace XBot::Cartesian::Planning {

ompl::base::RealVectorBounds boundsEigenToOmpl(Eigen::VectorXd qmin, Eigen::VectorXd qmax);

Eigen::VectorXd vectorToEigen(std::vector<double> v);

class StateSpace::Impl {

public:

    Impl();

    int addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id = "");

    int addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id = "");

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getBounds() const;

    std::unique_ptr<ompl::base::State> createState();

    void setValue(ompl::base::State& s,
                  Eigen::VectorXd q);

    Eigen::VectorXd getValue(const ompl::base::State& s);

    int addOmplSpace(ompl::base::StateSpacePtr ss, std::string id);

private:

    std::map<std::string, ompl::base::StateSpacePtr> _ss_map;
    std::vector<ompl::base::StateSpacePtr> _ss_vec;
    std::shared_ptr<ompl::base::CompoundStateSpace> _ss;

};

class RobotConfigurationSpace : public ompl::base::StateSpace
{

public:

    RobotConfigurationSpace(ModelInterface::ConstPtr model);

    ModelInterface::ConstPtr model() const;

    // StateSpace interface
    unsigned int getDimension() const override;
    double getMaximumExtent() const override;
    double getMeasure() const override;
    void enforceBounds(ompl::base::State *state) const override;
    bool satisfiesBounds(const ompl::base::State *state) const override;
    void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;
    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;
    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;
    void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;
    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
    ompl::base::State *allocState() const override;
    void freeState(ompl::base::State *state) const override;

    // State type
    class StateType : public ompl::base::State
    {
    public:
        Eigen::VectorXd q;
        ~StateType() = default;
    };

    // State sampler
    class StateSampler : public ompl::base::StateSampler
    {
        // StateSampler interface
    public:
        void sampleUniform(ompl::base::State *state) override;
        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

        ModelInterface::ConstPtr _model;
    };

private:

    ModelInterface::ConstPtr _model;

    static const Eigen::VectorXd& getQ(const ompl::base::State *);
    static Eigen::VectorXd& getQ(ompl::base::State *);

};


}

#endif // STATE_SPACE_HXX
