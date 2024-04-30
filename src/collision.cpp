#include <cartesio_planning/state_validity_checker/collision.h>

#include "impl/profiling.hxx"

using namespace XBot::Cartesian::Planning;

class CollisionValidityChecker::Impl
{

public:

    XBot::Collision::CollisionModel::Ptr collision_model;

    double threshold = 0.0;

    bool include_env = true;

    XBot::Collision::CollisionModel::ComputeCollisionFreeOptions opt;

    Eigen::VectorXd last_q;

    std::vector<int> coll_pair_ids;

    Collision::CollisionModel::LinkPairVector lpv;

    Callback cb;

private:

};

CollisionValidityChecker::CollisionValidityChecker(StateSpace::ConstPtr space,
                                                   XBot::Collision::CollisionModel::Ptr collision_model,
                                                   std::optional<std::string> id,
                                                   int substate_idx):
    StateValidityChecker(space, id ? *id : "collision", substate_idx)
{
    impl = std::make_unique<Impl>();

    impl->collision_model = collision_model;

    auto model = space->getModel(_idx < 0 ? 0 : _idx);

    if(!model)
    {
        throw std::runtime_error("could not get model from state at idx " + std::to_string(_idx));
    }

    impl->opt.w_norm = model->computeInertiaMatrix().diagonal().cwiseInverse().cwiseSqrt();

    impl->cb = [](auto){};

}

void CollisionValidityChecker::setIncludeEnvironment(bool flag)
{

}

void CollisionValidityChecker::setThreshold(double threshold)
{

}

bool CollisionValidityChecker::checkValid(const Eigen::VectorXd &q,
                                          std::optional<Eigen::VectorXd> &qnear) const
{

    TIKTOK(collision_check);

    impl->collision_model->update();

    impl->coll_pair_ids.clear();

    impl->lpv.clear();

    bool ret = !impl->collision_model->checkCollision(impl->coll_pair_ids,
                                                      impl->include_env,
                                                      impl->threshold);

    if(ret)
    {
        impl->cb(impl->lpv);
        return true;
    }

    // TIKTOK(collision_near);

    // Eigen::VectorXd qnear_tmp = q;

    // if(impl->collision_model->computeCollisionFree(qnear_tmp, impl->opt))
    // {
    //     TIKTOK(collision_near_ok);
    //     qnear = qnear_tmp;
    // }

    const auto& lp = impl->collision_model->getCollisionPairs(impl->include_env);

    for(auto i : impl->coll_pair_ids)
    {
        impl->lpv.push_back(lp[i]);
    }

    impl->cb(impl->lpv);

    return false;
}

void CollisionValidityChecker::printInvalidStateInformation(std::ostream &os) const
{
    os << "[";

    for(const auto& lp : impl->lpv)
    {
        os << lp.first << " vs " << lp.second << ", ";
    }

     os << "]";
}

void CollisionValidityChecker::setCallback(Callback cb)
{
    impl->cb = cb;
}

CollisionValidityChecker::~CollisionValidityChecker()
{

}
