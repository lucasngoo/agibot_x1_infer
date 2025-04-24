#include "control_module/keyboard_joint_controller.h"
#include <iostream>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace xyber_x1_infer::rl_control_module {

// Helper function to set up non-blocking keyboard input
void SetupNonBlockingKeyboard() {
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);
  ttystate.c_lflag &= ~(ICANON | ECHO);
  ttystate.c_cc[VMIN] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
  fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
}

// Helper function to restore terminal settings
void RestoreKeyboard() {
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);
  ttystate.c_lflag |= ICANON | ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

KeyboardJointController::KeyboardJointController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles) {}

KeyboardJointController::~KeyboardJointController() {
  running_ = false;
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
  RestoreKeyboard();
}

void KeyboardJointController::Init(const YAML::Node &cfg_node) {
  // Initialize joint names from config
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  
  // Initialize joint positions to zeros
  joint_positions_.resize(joint_names_.size(), 0.0);
  
  // Set up the joint_state_data structure
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);
  
  // Initialize stiffness and damping
  joint_conf_.init_state = Eigen::Map<vector_t>(
      cfg_node["init_state"].as<std::vector<double>>().data(), 
      cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(
      cfg_node["stiffness"].as<std::vector<double>>().data(), 
      cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(
      cfg_node["damping"].as<std::vector<double>>().data(), 
      cfg_node["damping"].as<std::vector<double>>().size());
  
  // Configure angle increment if provided
  if (cfg_node["angle_increment"]) {
    angle_increment_ = cfg_node["angle_increment"].as<double>();
  }
  
  // Set up keyboard mappings for left arm joints
  // Find indices for the 7 left arm joints
  int shoulder_pitch_idx = -1, shoulder_roll_idx = -1, shoulder_yaw_idx = -1;
  int elbow_pitch_idx = -1, elbow_yaw_idx = -1;
  int wrist_pitch_idx = -1, wrist_roll_idx = -1;
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "left_shoulder_pitch_joint") shoulder_pitch_idx = i;
    else if (joint_names_[i] == "left_shoulder_roll_joint") shoulder_roll_idx = i;
    else if (joint_names_[i] == "left_shoulder_yaw_joint") shoulder_yaw_idx = i;
    else if (joint_names_[i] == "left_elbow_pitch_joint") elbow_pitch_idx = i;
    else if (joint_names_[i] == "left_elbow_yaw_joint") elbow_yaw_idx = i;
    else if (joint_names_[i] == "left_wrist_pitch_joint") wrist_pitch_idx = i;
    else if (joint_names_[i] == "left_wrist_roll_joint") wrist_roll_idx = i;
  }
  
  // Define key mappings if joints were found
  if (shoulder_pitch_idx >= 0) {
    key_mappings_.push_back({'q', shoulder_pitch_idx, 1.0});  // Increase
    key_mappings_.push_back({'a', shoulder_pitch_idx, -1.0}); // Decrease
  }
  if (shoulder_roll_idx >= 0) {
    key_mappings_.push_back({'w', shoulder_roll_idx, 1.0});
    key_mappings_.push_back({'s', shoulder_roll_idx, -1.0});
  }
  if (shoulder_yaw_idx >= 0) {
    key_mappings_.push_back({'e', shoulder_yaw_idx, 1.0});
    key_mappings_.push_back({'d', shoulder_yaw_idx, -1.0});
  }
  if (elbow_pitch_idx >= 0) {
    key_mappings_.push_back({'r', elbow_pitch_idx, 1.0});
    key_mappings_.push_back({'f', elbow_pitch_idx, -1.0});
  }
  if (elbow_yaw_idx >= 0) {
    key_mappings_.push_back({'t', elbow_yaw_idx, 1.0});
    key_mappings_.push_back({'g', elbow_yaw_idx, -1.0});
  }
  if (wrist_pitch_idx >= 0) {
    key_mappings_.push_back({'y', wrist_pitch_idx, 1.0});
    key_mappings_.push_back({'h', wrist_pitch_idx, -1.0});
  }
  if (wrist_roll_idx >= 0) {
    key_mappings_.push_back({'u', wrist_roll_idx, 1.0});
    key_mappings_.push_back({'j', wrist_roll_idx, -1.0});
  }
  
  // Start keyboard input thread
  SetupNonBlockingKeyboard();
  keyboard_thread_ = std::thread(&KeyboardJointController::KeyboardInputThread, this);
}

void KeyboardJointController::RestartController() {
  // Reset joint positions from current state
  std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joint_positions_[i] = joint_state_data_.position[i];
  }
}

void KeyboardJointController::Update() {
  // Copy current joint states to our joint_positions_ for newly tracked joints
  std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    // Only copy position if not already modified by keyboard input
    if (std::abs(joint_positions_[i] - 0.0) < 0.0001) {
      joint_positions_[i] = joint_state_data_.position[i];
    }
  }
  
  // Process any key presses to modify joint positions
  std::lock_guard<std::mutex> keys_lock(keys_mutex_);
  for (const auto& mapping : key_mappings_) {
    if (pressed_keys_[mapping.key]) {
      joint_positions_[mapping.joint_index] += mapping.direction * angle_increment_;
    }
  }
}

my_ros2_proto::msg::JointCommand KeyboardJointController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joint_cmd.position[i] = joint_positions_[i];
    joint_cmd.velocity[i] = 0.0;
    joint_cmd.effort[i] = 0.0;
    joint_cmd.stiffness[i] = joint_conf_.stiffness(i);
    joint_cmd.damping[i] = joint_conf_.damping(i);
  }
  
  return joint_cmd;
}

void KeyboardJointController::KeyboardInputThread() {
  char ch;
  
  while (running_) {
    // Clear all keys first
    {
      std::lock_guard<std::mutex> lock(keys_mutex_);
      for (const auto& mapping : key_mappings_) {
        pressed_keys_[mapping.key] = false;
      }
    }
    
    // Read any available key presses
    while (read(STDIN_FILENO, &ch, 1) > 0) {
      std::lock_guard<std::mutex> lock(keys_mutex_);
      
      // Mark this key as pressed
      for (const auto& mapping : key_mappings_) {
        if (mapping.key == ch) {
          pressed_keys_[ch] = true;
          break;
        }
      }
    }
    
    // Sleep a bit to avoid busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

} // namespace xyber_x1_infer::rl_control_module 