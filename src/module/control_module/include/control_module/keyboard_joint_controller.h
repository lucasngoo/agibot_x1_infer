#pragma once
#include <memory>
#include <unordered_map>
#include <mutex>
#include <thread>
#include "control_module/controller_base.h"

namespace xyber_x1_infer::rl_control_module {

class KeyboardJointController : public ControllerBase {
 public:
  KeyboardJointController(const bool use_sim_handles);
  ~KeyboardJointController();

  void Init(const YAML::Node &cfg_node) override;
  void RestartController() override;
  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

 private:
  // Thread for keyboard input
  void KeyboardInputThread();
  
  // Current joint positions to command
  std::vector<double> joint_positions_;
  
  // Mapping of keyboard keys to joint indices and directions
  struct KeyJointMapping {
    char key;
    int joint_index;
    double direction; // 1.0 for increase, -1.0 for decrease
  };
  std::vector<KeyJointMapping> key_mappings_;
  
  // Thread for reading keyboard input
  std::thread keyboard_thread_;
  std::atomic<bool> running_{true};
  
  // Increment amount per keystroke
  double angle_increment_ = 0.005;
  
  // Store pressed keys
  std::mutex keys_mutex_;
  std::unordered_map<char, bool> pressed_keys_;
};

} // namespace xyber_x1_infer::rl_control_module 