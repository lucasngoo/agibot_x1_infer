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
  
  // Structure to hold joint indices for each arm
  struct JointIndices {
    int shoulder_pitch;
    int elbow_pitch;
    int wrist_pitch;
    int wrist_roll;
  };
  
  // Initialize joint indices for both arms
  void InitJointIndices();
  
  // Update key mappings based on current arm selection
  void UpdateKeyMappings();
  
  // Current joint positions to command
  std::vector<double> joint_positions_;
  
  // Mapping of keyboard keys to joint indices and directions
  struct KeyJointMapping {
    char key;
    int joint_index;
    double direction; // 1.0 for increase, -1.0 for decrease
  };
  std::vector<KeyJointMapping> key_mappings_;
  
  // Joint indices for left and right arms
  JointIndices left_indices_;
  JointIndices right_indices_;
  int lumbar_yaw_idx_ = -1;
  
  // Flag to determine which arm is currently being controlled
  bool use_left_arm_ = true;
  
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