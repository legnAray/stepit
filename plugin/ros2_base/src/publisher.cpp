#include <algorithm>
#include <stepit/ros2/node.h>
#include <stepit/ros2/publisher.h>

namespace stepit {
Ros2Publisher::Ros2Publisher() {
  auto node = getNode();
  if (publisher::g_filter.publish_status) {
    status_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", getDefaultQoS());
  }
  if (publisher::g_filter.publish_low_level) {
    imu_pub_   = node->create_publisher<sensor_msgs::msg::Imu>("imu", getDefaultQoS());
    joint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", getDefaultQoS());
  }
}

void Ros2Publisher::publishStatus() {
  if (not status_pub_) return;
  auto statuses     = getStatusSnapshot();
  status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status_msg_.name  = "stepit";
  status_msg_.values.clear();
  for (const auto &item : statuses) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key   = item.first;
    kv.value = item.second;
    status_msg_.values.push_back(kv);
  }
  status_pub_->publish(status_msg_);
}

void Ros2Publisher::publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {
  if (not imu_pub_ or not joint_pub_) return;
  auto timestamp                 = getNode()->now();
  imu_msg_.header.stamp          = timestamp;
  imu_msg_.angular_velocity.x    = state.imu.gyroscope[0];
  imu_msg_.angular_velocity.y    = state.imu.gyroscope[1];
  imu_msg_.angular_velocity.z    = state.imu.gyroscope[2];
  imu_msg_.linear_acceleration.x = state.imu.accelerometer[0];
  imu_msg_.linear_acceleration.y = state.imu.accelerometer[1];
  imu_msg_.linear_acceleration.z = state.imu.accelerometer[2];
  imu_msg_.orientation.w         = state.imu.quaternion[0];
  imu_msg_.orientation.x         = state.imu.quaternion[1];
  imu_msg_.orientation.y         = state.imu.quaternion[2];
  imu_msg_.orientation.z         = state.imu.quaternion[3];
  imu_pub_->publish(imu_msg_);

  joint_msg_.header.frame_id = spec.robot_name;
  joint_msg_.header.stamp    = timestamp;
  std::size_t msg_dim        = 3 * spec.dof + spec.foot_names.size();
  joint_msg_.position.resize(msg_dim);
  joint_msg_.velocity.resize(msg_dim);
  joint_msg_.effort.resize(msg_dim);
  joint_msg_.name.resize(msg_dim);

  std::size_t idx{};
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_state  = state.motor_state[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_joint";
    joint_msg_.position[idx] = joint_state.q;
    joint_msg_.velocity[idx] = joint_state.dq;
    joint_msg_.effort[idx]   = joint_state.tor;
  }
  for (std::size_t i{}; i < spec.foot_names.size(); ++i, ++idx) {
    joint_msg_.name[idx]     = spec.foot_names[i];
    joint_msg_.position[idx] = 0.0;
    joint_msg_.velocity[idx] = 0.0;
    joint_msg_.effort[idx]   = state.foot_force[i];
  }
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_cmd    = cmd[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_cmd";
    joint_msg_.position[idx] = joint_cmd.q;
    joint_msg_.velocity[idx] = joint_cmd.dq;
    joint_msg_.effort[idx]   = joint_cmd.tor;
  }
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_state  = state.motor_state[i];
    const auto &joint_cmd    = cmd[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_gain";
    joint_msg_.position[idx] = joint_cmd.Kp;
    joint_msg_.velocity[idx] = joint_cmd.Kd;
    joint_msg_.effort[idx]   =  // desired torque
        (joint_cmd.q - joint_state.q) * joint_cmd.Kp + (joint_cmd.dq - joint_state.dq) * joint_cmd.Kd + joint_cmd.tor;
  }
  joint_pub_->publish(joint_msg_);
}

void Ros2Publisher::publishArray(const std::string &name, cArrXf vec) {
  auto channel = pub_map_.find(name);
  if (channel == pub_map_.end()) {
    pub_map_[name] = getNode()->create_publisher<std_msgs::msg::Float32MultiArray>(name, 1);
    channel        = pub_map_.find(name);
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(vec.size());
  std::copy_n(vec.data(), vec.size(), msg.data.begin());
  channel->second->publish(msg);
}

STEPIT_REGISTER_PUBLISHER(ros2, kDefPriority, Publisher::make<Ros2Publisher>);
}  // namespace stepit
