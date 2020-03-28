#ifndef PLANNING_DATA_H
#define PLANNING_DATA_H

#include <ompl/base/PlannerData.h>
#include <matlogger2/mat_data.h>
#include <matlogger2/matlogger2.h>
#include "state_wrapper.h"

XBot::matlogger2::MatData PlannerDataToMatMata(const ompl::base::PlannerData& pdata,
                                               const XBot::Cartesian::Planning::StateWrapper& sw)
{
    using XBot::matlogger2::MatData;

    auto md = MatData::make_struct();

    Eigen::VectorXd state;
    sw.getState(pdata.getVertex(0).getState(),
                state);

    Eigen::MatrixXd V(state.size(), pdata.numVertices());

    std::vector<uint> edge_list;

    auto edges = MatData::make_cell(pdata.numVertices());

    for(int i = 0; i < pdata.numVertices(); i++)
    {
        sw.getState(pdata.getVertex(i).getState(),
                    state);

        V.col(i) = state;

        pdata.getEdges(i, edge_list);

        Eigen::VectorXd edge_list_eig = Eigen::Matrix<uint, -1, 1>::Map(edge_list.data(),
                                                                        edge_list.size()).cast<double>();

        edges[i] = edge_list_eig;
    }

    md["vertices"] = V;
    md["edges"] = edges;

    return md;

}

#endif // PLANNING_DATA_H
