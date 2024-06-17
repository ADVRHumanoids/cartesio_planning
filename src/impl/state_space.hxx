#ifndef STATE_SPACE_HXX
#define STATE_SPACE_HXX

#include <cartesio_planning/state_space.h>

#include <ompl/base/StateSpace.h>

#include <ompl/base/ScopedState.h>

#include <ompl/base/StateSpaceTypes.h>

#include "../ompl_replacement/AtlasStateSpace.h"

#include <yaml-cpp/yaml.h>

#include <cartesio_planning/state_validity_checker.h>

namespace XBot::Cartesian::Planning {

ompl::base::RealVectorBounds boundsEigenToOmpl(Eigen::VectorXd qmin, Eigen::VectorXd qmax);

Eigen::VectorXd vectorToEigen(std::vector<double> v);

class StateSpace::Impl {

public:

    Impl();

    void setOptions(YAML::Node options);

    int addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id = "");

    int addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id = "");

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getBounds() const;

    ompl::base::State * createState() const;

    void setValue(ompl::base::State& s,
                  Eigen::VectorXd q) const;

    Eigen::VectorXd getValue(const ompl::base::State& s) const;

    int addOmplSpace(ompl::base::StateSpacePtr ss, std::string id, Type type);

    ompl::base::StateSpacePtr getStateSpace() const;

    ompl::base::StateSpacePtr getAmbientStateSpace() const;

    int getNq() const;

    int getNv() const;

    int getNq(int i) const;

    int getQIndex(int i) const;

    ModelInterface::Ptr getModel(int i) const;

    void updateModelState(const Eigen::VectorXd& q) const;

    void setConstraint(Constraint::Ptr c);

    Constraint::Ptr getConstraint() const;

    Eigen::VectorXd ambientSum(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

    Eigen::VectorXd ambientDiff(const Eigen::VectorXd& q1, const Eigen::VectorXd& q0);

    Eigen::VectorXd interpolate(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, double tau);

    void setSpaceInformation(ompl::base::SpaceInformation * si);

    Eigen::VectorXd ambientRandom() const;

    Eigen::VectorXd random() const;

    bool addStateValidityChecker(StateValidityChecker::ConstPtr svc);

    bool checkBounds(const Eigen::VectorXd& q) const;

    bool checkValid(const Eigen::VectorXd &q,
                    bool force_verbose = false,
                    std::ostream &os = std::cout,
                    std::vector<std::string> *failed_checks = nullptr) const;

    bool isStateValid(const ompl::base::State &state,
                      bool force_verbose = false,
                      std::ostream &os = std::cout,
                      std::vector<std::string> *failed_checks = nullptr) const;

private:
    using BinaryVectorOp = ompl::base::AtlasStateSpaceNE::BinaryVectorOp;

    std::map<std::string, ompl::base::StateSpacePtr> _ss_map;
    std::vector<ompl::base::StateSpacePtr> _ss_vec;
    std::vector<int> _q_index;
    std::vector<int> _v_index, _nv;
    int _ss_nv;
    std::vector<BinaryVectorOp> _f_sum, _f_diff;

    ompl::base::StateSpacePtr _ss;
    std::shared_ptr<ompl::base::CompoundStateSpace> _ss_comp;
    Constraint::Ptr _constr;

    std::map<std::string, StateValidityChecker::ConstPtr> _svc_map;

    YAML::Node _options;

};

class RobotConfigurationSpace : public ompl::base::StateSpace
{

public:

    RobotConfigurationSpace(ModelInterface::Ptr model,
                            Planning::StateSpace::RobotConfigurationSpaceOptions opt);

    ModelInterface::Ptr model() const;

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
    void printState(const ompl::base::State *state, std::ostream &out) const override;
    double *getValueAddressAtIndex(ompl::base::State *state, unsigned int index) const override;


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

        StateSampler(const StateSpace *space,
                     ModelInterface::ConstPtr model,
                     Planning::StateSpace::RobotConfigurationSpaceOptions opt);

        void sampleUniform(ompl::base::State *state) override;
        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

        ModelInterface::ConstPtr _model;
        Planning::StateSpace::RobotConfigurationSpaceOptions _opt;
    };

private:

    ModelInterface::Ptr _model;

    Planning::StateSpace::RobotConfigurationSpaceOptions _opt;

    static const Eigen::VectorXd& getQ(const ompl::base::State *);

    static Eigen::VectorXd& getQ(ompl::base::State *);

};

}

#endif // STATE_SPACE_HXX
