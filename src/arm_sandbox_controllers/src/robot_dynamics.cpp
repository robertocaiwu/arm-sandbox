#include "arm_sandbox_controllers/robot_dynamics.hpp"

#include <algorithm>
#include <stdexcept>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace arm_sandbox_controllers
{

RobotDynamics::RobotDynamics(
  const std::string & urdf_xml, const std::vector<std::string> & arm_joints, const Eigen::VectorXd & armature)
{
  pinocchio::Model full;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, full);
  } catch (const std::exception & error) {
    throw std::invalid_argument(std::string("RobotDynamics: the URDF does not parse: ") + error.what());
  }
  for (const auto & name : arm_joints) {
    if (!full.existJointName(name)) {
      throw std::invalid_argument("RobotDynamics: arm joint '" + name + "' not in the URDF");
    }
  }

  // Lock every joint that isn't an arm joint (e.g. the fingers) at its neutral position.
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); ++id) {
    if (std::find(arm_joints.begin(), arm_joints.end(), full.names[id]) == arm_joints.end()) {
      locked.push_back(id);
    }
  }
  model_ = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));

  const std::vector<std::string> reduced_names(model_.names.begin() + 1, model_.names.end());
  if (reduced_names != arm_joints) {
    throw std::invalid_argument("RobotDynamics: the URDF's joint order differs from arm_joints");
  }
  if (armature.size() != model_.nv) {
    throw std::invalid_argument("RobotDynamics: armature needs one value per arm joint");
  }
  model_.armature = armature;
  data_ = pinocchio::Data(model_);
  mass_matrix_ = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  effort_limits_ = model_.effortLimit;
}

void RobotDynamics::update(const Eigen::VectorXd & q, const Eigen::VectorXd & qd)
{
  pinocchio::crba(model_, data_, q);  // fills the upper triangle of data_.M (armature included)
  mass_matrix_.triangularView<Eigen::Upper>() = data_.M.triangularView<Eigen::Upper>();
  mass_matrix_.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model_, data_, q, qd);
  pinocchio::computeGeneralizedGravity(model_, data_, q);
}

const Eigen::VectorXd & RobotDynamics::forward_dynamics(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::VectorXd & tau)
{
  return pinocchio::aba(model_, data_, q, qd, tau);
}

}  // namespace arm_sandbox_controllers
