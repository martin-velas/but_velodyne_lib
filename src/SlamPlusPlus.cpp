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

#include <iostream>

#include <but_velodyne/SlamPlusPlus.h>

using namespace std;

namespace but_velodyne {

void SlamPlusPlus::addEdge(const int src_i, const int trg_i, const Eigen::Affine3f &T,
                           const Eigen::Matrix<double, 6, 6> &information) {

  Eigen::Matrix<float, 6, 1> T_coeff;
  T_coeff.head(3) = T.translation();
  Eigen::AngleAxisf R(T.rotation());
  T_coeff.tail(3) = R.axis() * R.angle();

  system.r_Add_Edge(CEdgePose3D(src_i, trg_i, T_coeff.cast<double>(), information, system));
}

void SlamPlusPlus::optimize(vector<Eigen::Affine3f> &poses, vector<Eigen::Vector3f> &features) {
  cerr << "========== SLAM plus plus optimization ==========" << endl << flush;
  cerr << "initial denormalized chi2 error: " << solver.f_Chi_Squared_Error_Denorm() << endl << flush;
  solver.Optimize();
  cerr << "=================== * DONE * ====================" << endl << flush;

  const CSystemType::_TyVertexMultiPool &vertex_pool = system.r_Vertex_Pool();

  for (size_t i = 0, n = vertex_pool.n_Size(); i < n; ++i) {
    Eigen::Map<const Eigen::VectorXd> v_state = vertex_pool[i].v_State();

    if (v_state.rows() >= 3) {
      Eigen::Vector3f t_coeff = v_state.head(3).cast<float>();
      Eigen::Translation3f t(t_coeff);
      if (v_state.rows() == 3) {
        features.push_back(t_coeff);
      } else if (v_state.rows() == 6) {
        Eigen::Vector3f R_axis = v_state.tail(3).cast<float>();
        const float R_angle = R_axis.norm();
        if (R_angle < 1e-12) {
          poses.push_back(Eigen::Affine3f(t));
        } else {
          R_axis.normalize();
          Eigen::AngleAxisf R(R_angle, R_axis);
          poses.push_back(t * R);
        }
      } else {
        cerr << "Unknown state of graph element - not vertex nor pose. Ignoring." << endl;
      }
    }
  }
}

}
