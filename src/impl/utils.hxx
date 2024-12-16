#ifndef UTILS_HXX
#define UTILS_HXX

#endif // UTILS_HXX

#include <Eigen/Dense>
#include <vector>

namespace utils {

/**
 * @brief eigenToStd
 * @param qeig
 * @param qvec
 */
void eigenToStd(Eigen::Ref<const Eigen::VectorXd> qeig,
                std::vector<double>& qvec)
{
    qvec.resize(qeig.size());

    Eigen::VectorXd::Map(qvec.data(), qvec.size()) = qeig;
}


}
