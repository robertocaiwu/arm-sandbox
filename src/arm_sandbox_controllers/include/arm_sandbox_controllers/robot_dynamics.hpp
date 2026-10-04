#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace arm_sandbox_controllers
{

/// Joint-space dynamics of the arm, M(q) q'' + C(q, q') q' + g(q) = tau, from Pinocchio on the URDF.
///
/// The model is reduced to `arm_joints` (other joints, e.g. the fingers, are locked at zero) and
/// gets the actuators' `armature` (reflected rotor inertia) added to the diagonal of M, as the
/// MJCF does. ROS-free. After construction, `update` and `forward_dynamics` never allocate.
class RobotDynamics
{
public:
  /// Throws std::invalid_argument if the URDF doesn't parse, an arm joint is missing, the reduced
  /// model's joint order differs from `arm_joints`, or `armature` has the wrong size.
  RobotDynamics(
    const std::string & urdf_xml, const std::vector<std::string> & arm_joints, const Eigen::VectorXd & armature);

  std::size_t num_joints() const { return static_cast<std::size_t>(model_.nv); }
  /// URDF effort limits of the arm joints (N m or N).
  const Eigen::VectorXd & effort_limits() const { return effort_limits_; }

  /// Computes M(q), C(q, q') q' + g(q) and g(q). Read them with the getters below.
  void update(const Eigen::VectorXd & q, const Eigen::VectorXd & qd);
  const Eigen::MatrixXd & mass_matrix() const { return mass_matrix_; }  ///< symmetric
  const Eigen::VectorXd & nonlinear_effects() const { return data_.nle; }  ///< C q' + g
  const Eigen::VectorXd & gravity() const { return data_.g; }

  /// q'' for torques `tau` (articulated-body algorithm). Used as the simulated plant in tests.
  const Eigen::VectorXd & forward_dynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::VectorXd & tau);

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::MatrixXd mass_matrix_;
  Eigen::VectorXd effort_limits_;
};

}  // namespace arm_sandbox_controllers
