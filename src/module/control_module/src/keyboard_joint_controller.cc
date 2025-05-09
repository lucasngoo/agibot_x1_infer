#include "control_module/keyboard_joint_controller.h"
#include <iostream>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

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
    : ControllerBase(use_sim_handles), use_left_arm_(false) {
  // Initialize pose vectors
  current_pose_.resize(9, 0.0);  // x,y,z,qx,qy,qz,qw,button,toggle
  previous_pose_.resize(9, 0.0);
}

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
  
  // Initialize joint indices for both arms
  InitJointIndices();
  
  // Set up initial key mappings for left arm (default)
  UpdateKeyMappings();
  
  // Start keyboard input thread
  SetupNonBlockingKeyboard();
  keyboard_thread_ = std::thread(&KeyboardJointController::KeyboardInputThread, this);
  
  std::cout << "Keyboard Joint Controller initialized. Using left arm. Press 'z' to toggle arms." << std::endl;
  
  // Initialize pose data
  ReadPoseData();
}

void KeyboardJointController::InitJointIndices() {
  // Find indices for joints on both sides
  left_indices_.shoulder_pitch = -1;
  left_indices_.shoulder_yaw = -1;
  left_indices_.shoulder_roll = -1;
  left_indices_.elbow_pitch = -1;
  left_indices_.wrist_roll = -1;
  left_indices_.elbow_yaw = -1;
  
  right_indices_.shoulder_pitch = -1;
  right_indices_.shoulder_yaw = -1;
  right_indices_.shoulder_roll = -1;
  right_indices_.elbow_pitch = -1;
  right_indices_.wrist_roll = -1;
  right_indices_.elbow_yaw = -1;
  
  lumbar_yaw_idx_ = -1;
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    // Left arm joints
    if (joint_names_[i] == "left_shoulder_pitch_joint") left_indices_.shoulder_pitch = i;
    else if (joint_names_[i] == "left_shoulder_yaw_joint") left_indices_.shoulder_yaw = i;
    else if (joint_names_[i] == "left_shoulder_roll_joint") left_indices_.shoulder_roll = i;
    else if (joint_names_[i] == "left_elbow_pitch_joint") left_indices_.elbow_pitch = i;
    else if (joint_names_[i] == "left_wrist_roll_joint") left_indices_.wrist_roll = i;
    else if (joint_names_[i] == "left_elbow_yaw_joint") left_indices_.elbow_yaw = i;
    
    // Right arm joints
    else if (joint_names_[i] == "right_shoulder_pitch_joint") right_indices_.shoulder_pitch = i;
    else if (joint_names_[i] == "right_shoulder_yaw_joint") right_indices_.shoulder_yaw = i;
    else if (joint_names_[i] == "right_shoulder_roll_joint") right_indices_.shoulder_roll = i;
    else if (joint_names_[i] == "right_elbow_pitch_joint") right_indices_.elbow_pitch = i;
    else if (joint_names_[i] == "right_wrist_roll_joint") right_indices_.wrist_roll = i;
    else if (joint_names_[i] == "right_elbow_yaw_joint") right_indices_.elbow_yaw = i;
    
    // Lumbar joint
    else if (joint_names_[i] == "lumbar_yaw_joint") lumbar_yaw_idx_ = i;
  }
}

void KeyboardJointController::UpdateKeyMappings() {
  // Clear existing mappings
  key_mappings_.clear();
  
  // Get current arm's indices
  const JointIndices& indices = use_left_arm_ ? left_indices_ : right_indices_;
  
  // Create new mappings based on the requested controls
  // w/s - shoulder_pitch_joint down and up (reversed)
  if (indices.shoulder_pitch >= 0) {
    key_mappings_.push_back({'w', indices.shoulder_pitch, -1.0});  // Down
    key_mappings_.push_back({'s', indices.shoulder_pitch, 1.0});   // Up
  }
  
  // q/e - elbow_pitch_joint up and down
  if (indices.elbow_pitch >= 0) {
    key_mappings_.push_back({'q', indices.elbow_pitch, 1.0});  // Up
    key_mappings_.push_back({'e', indices.elbow_pitch, -1.0}); // Down
  }
  
  // i/k - wrist_roll_joint down and up (reversed)
  if (indices.wrist_roll >= 0) {
    key_mappings_.push_back({'i', indices.wrist_roll, -1.0});  // Down
    key_mappings_.push_back({'k', indices.wrist_roll, 1.0});   // Up
  }
  
  // j/l - elbow_yaw_joint control (direction depends on which arm is active)
  if (indices.elbow_yaw >= 0) {
    if (use_left_arm_) {
      // Left arm: j=up, l=down
      key_mappings_.push_back({'j', indices.elbow_yaw, 1.0});   // Up
      key_mappings_.push_back({'l', indices.elbow_yaw, -1.0});  // Down
    } else {
      // Right arm: j=down, l=up (reversed)
      key_mappings_.push_back({'j', indices.elbow_yaw, -1.0});  // Down
      key_mappings_.push_back({'l', indices.elbow_yaw, 1.0});   // Up
    }
  }
  
  // a/d - shoulder_yaw_joint control (direction depends on which arm is active)
  if (indices.shoulder_yaw >= 0) {
    if (use_left_arm_) {
      // Left arm: a=up, d=down
      key_mappings_.push_back({'a', indices.shoulder_yaw, 1.0});   // Up
      key_mappings_.push_back({'d', indices.shoulder_yaw, -1.0});  // Down
    } else {
      // Right arm: a=down, d=up (reversed)
      key_mappings_.push_back({'a', indices.shoulder_yaw, -1.0});  // Down
      key_mappings_.push_back({'d', indices.shoulder_yaw, 1.0});   // Up
    }
  }

  // u/o - shoulder_roll_joint control (direction depends on which arm is active)
  if (indices.shoulder_roll >= 0) {
    if (use_left_arm_) {
      // Left arm: u=down, o=up (reversed)
      key_mappings_.push_back({'u', indices.shoulder_roll, -1.0});   // Down
      key_mappings_.push_back({'o', indices.shoulder_roll, 1.0});  // Up
    } else {
      // Right arm: u=up, o=down
      key_mappings_.push_back({'u', indices.shoulder_roll, 1.0});  // Up
      key_mappings_.push_back({'o', indices.shoulder_roll, -1.0});   // Down
    }
  }
  
  // x/c - lumbar_yaw_joint up and down (this is shared, not arm-specific)
  if (lumbar_yaw_idx_ >= 0) {
    key_mappings_.push_back({'x', lumbar_yaw_idx_, 1.0});  // Up
    key_mappings_.push_back({'c', lumbar_yaw_idx_, -1.0}); // Down
  }
}

void KeyboardJointController::RestartController() {
  // Reset joint positions from current state
  std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joint_positions_[i] = joint_state_data_.position[i];
  }
}

void KeyboardJointController::Update() {
  // Read latest pose data
  ReadPoseData();
  
  // Check for significant z movement
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!current_pose_.empty() && !previous_pose_.empty()) {
      double x_diff = current_pose_[0] - previous_pose_[0];
      double y_diff = current_pose_[1] - previous_pose_[1];
      double z_diff = current_pose_[2] - previous_pose_[2];
      bool button_pressed = current_pose_[7];
      
      // If x movement exceeds threshold, simulate w/s/q/e key press
      if (std::abs(x_diff) > movement_threshold_) {
        std::lock_guard<std::mutex> keys_lock(keys_mutex_);
        if (x_diff > 0) {
          // Moving forward
          if (button_pressed) {
            pressed_keys_['q'] = true;
	  } else {
            pressed_keys_['w'] = true;
          }
        } else {
          // Moving backward
          if (button_pressed) {
            pressed_keys_['q'] = false;
          } else {
            pressed_keys_['w'] = false;
          }
        }
        if (button_pressed) {
          pressed_keys_['e'] = !pressed_keys_['q'];
        } else {
          pressed_keys_['s'] = !pressed_keys_['w'];
        }
      }

      // If y movement exceeds threshold, simulate x/c/j/l key press
      if (std::abs(y_diff) > movement_threshold_) {
        std::lock_guard<std::mutex> keys_lock(keys_mutex_);
        if (y_diff > 0) {
          // Moving right
          if (button_pressed) {
            pressed_keys_['j'] = true;
          } else {
            pressed_keys_['x'] = true;
          }
        } else {
          // Moving left
          if (button_pressed) {
            pressed_keys_['j'] = false;
          } else {
            pressed_keys_['x'] = false;
          }
        }
	if (button_pressed) {
          pressed_keys_['l'] = !pressed_keys_['j'];
	} else {
          pressed_keys_['c'] = !pressed_keys_['x'];
        }
      }

      // If z movement exceeds threshold, simulate u/o/i/k key press
      if (std::abs(z_diff) > movement_threshold_) {
        std::lock_guard<std::mutex> keys_lock(keys_mutex_);
        if (z_diff > 0) {
          // Moving up
          if (button_pressed) {
            pressed_keys_['i'] = true;
          } else {
            pressed_keys_['u'] = use_left_arm_ ? true : false;
          }
        } else {
          // Moving down
          if (button_pressed) {
            pressed_keys_['i'] = false;
          } else {
            pressed_keys_['u'] = use_left_arm_ ? false : true;
          }
        }
        if (button_pressed) {
          pressed_keys_['k'] = !pressed_keys_['i'];
        } else {
          pressed_keys_['o'] = !pressed_keys_['u'];
        }
      }
    }
  }
  
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
      
      // Check for arm toggle key 'z'
      if (ch == 'z') {
        use_left_arm_ = !use_left_arm_;
        UpdateKeyMappings();
        std::cout << "Switched to " << (use_left_arm_ ? "left" : "right") << " arm controls." << std::endl;
        continue;
      }
      
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

// New method to read pose data from CSV
void KeyboardJointController::ReadPoseData() {
  std::string pose_path = std::string(getenv("HOME")) + "/pose.csv";
  std::ifstream file(pose_path);
  
  if (!file.is_open()) {
    return; // Can't open file, just return
  }
  
  std::string line;
  std::vector<std::string> rows;
  
  // Read all lines
  while (std::getline(file, line)) {
    rows.push_back(line);
  }
  
  // Need at least two rows (header and data)
  if (rows.size() < 2) {
    return;
  }
  
  // Parse data row
  std::string data_row = rows[1];
  std::stringstream ss(data_row);
  std::string value;
  std::vector<double> pose_data;
  
  while (std::getline(ss, value, ',')) {
    try {
      pose_data.push_back(std::stod(value));
    } catch (const std::exception& e) {
      pose_data.push_back(0.0);
    }
  }
  
  // Ensure we have at least 9 values (x,y,z,qx,qy,qz,qw,button,toggle)
  if (pose_data.size() >= 9) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    previous_pose_ = current_pose_;
    current_pose_ = pose_data;
  }
}

} // namespace xyber_x1_infer::rl_control_module 
