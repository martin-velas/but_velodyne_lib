/*
 * SLAM++ wrapper.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Date: 28/04/2020
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BUT_VELODYNE_LIB_SLAMPLUSPLUS_H
#define BUT_VELODYNE_LIB_SLAMPLUSPLUS_H

#include <slam/LinearSolver_UberBlock.h> // linear solver
#include <slam/ConfigSolvers.h> // nonlinear graph solvers
#include <slam/SE3_Types.h> // SE(3) types


namespace but_velodyne {

class SlamPlusPlus {

private:

    typedef MakeTypelist(CVertexPose3D, CVertexLandmark3D) TVertexTypelist;
    typedef MakeTypelist(CEdgePose3D, CEdgePoseLandmark3D) TEdgeTypelist;

    typedef CFlatSystem<CBaseVertex, TVertexTypelist, CBaseEdge, TEdgeTypelist> CSystemType;
    typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;

    typedef CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> CNonlinearSolverType;

    CSystemType system;

    CNonlinearSolverType solver;

public:

    SlamPlusPlus(const size_t max_iterations = 10, const bool verbose = true) :
            solver(system, 0, 0, max_iterations,
                   0.01, verbose, CLinearSolverType(), false) {
    }

    void addEdge(const int src_i, const int trg_i, const Eigen::Affine3f &T,
                 const Eigen::Matrix<double, 6, 6> &information);

    void optimize(std::vector<Eigen::Affine3f> &poses, std::vector<Eigen::Vector3f> &features);

    void optimize(std::vector<Eigen::Affine3f> &poses) {
      std::vector<Eigen::Vector3f> features_dummy;
      optimize(poses, features_dummy);
    }

};

}


#endif //BUT_VELODYNE_LIB_SLAMPLUSPLUS_H
