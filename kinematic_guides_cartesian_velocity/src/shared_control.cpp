#include "kinematic_guides_cartesian_velocity/shared_control.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <functional>
#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/msg/shared_control_debug.hpp"
#include "visual_servoing/msg/shared_control_goal_debug.hpp"
#include <unordered_map>
#include <unordered_set>


/*
 * Integration note:
 *
 * The visual_servoing package is responsible for:
 *   - camera topic configuration,
 *   - AprilTag detection,
 *   - camera-frame pose estimation,
 *   - TF transform to target_frame,
 *   - OpenCV display,
 *   - SAVE POSE button client,
 *   - SAVE POSE service/YAML persistence via saved_goal_manager_node.
 *
 * This controller is responsible only for:
 *   - consuming /visual_servoing/detected_goals,
 *   - inserting detected tags into active_goals_,
 *   - running the shared-control confidence/soft-goal logic,
 *   - publishing robot Cartesian velocity commands.
 */

namespace cartesian_velocity_controller
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SharedControlVelocityController::SharedControlVelocityController()
      : ControllerInterface(), latest_twist_(),
        current_orientation_(Eigen::Quaterniond::Identity()),
        current_position_(Eigen::Vector3d::Zero()),
        previous_cartesian_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_cartesian_angular_velocity_(Eigen::Vector3d::Zero()),
        previous_initial_filtered_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_initial_filtered_angular_velocity_(Eigen::Vector3d::Zero()),
        previous_filtered_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_filtered_angular_velocity_(Eigen::Vector3d::Zero()),
        enable_shared_control_assistance_(false), enable_velocity_saturation_(false),
        enable_debug_publish_(false), initial_filter_cutoff_frequency_(0.0),
        final_filter_cutoff_frequency_(0.0), lpf_initialized_(false), max_linear_delta_(0.0),
        max_angular_delta_(0.0)
  {
  }

  SharedControlVelocityController::~SharedControlVelocityController()
  {
  }

  controller_interface::InterfaceConfiguration SharedControlVelocityController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // Use command interface names provided by the robot interface

    config.names = robot_vel_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration SharedControlVelocityController::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    robot_vel_interface_->set_states_names();

    config.names = robot_vel_interface_->get_states_names();
    return config;
  }

  CallbackReturn SharedControlVelocityController::on_init()
  {
    // Subscribe to custom TeleopCmd messages
    teleop_cmd_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&SharedControlVelocityController::teleopCmdCallback, this,
                  std::placeholders::_1));

    goal_sub_ = get_node()->create_subscription<extender_msgs::msg::SharedControlGoalArray>(
        "/shared_control/dynamic_goals", 10,
        std::bind(&SharedControlVelocityController::goalCallback, this, std::placeholders::_1));
    
    // ---------------------------------------------------------------------------
    // Dynamic AprilTag goal subscriber
    // ---------------------------------------------------------------------------
    // visual_servoing_node publishes all currently visible tags as 6D goals in the
    // target robot frame on /visual_servoing/detected_goals.
    // This controller only consumes those already-transformed goals and updates
    // the shared-control goal set. It does not run AprilTag detection, TF camera
    // transforms, or OpenCV display; those are handled by the visual_servoing package.
    goals_sub_ = get_node()->create_subscription<visual_servoing::msg::DetectedGoalArray>(
        "/visual_servoing/detected_goals",
        10,
        std::bind(
            &SharedControlVelocityController::goalsCallback,
            this,
            std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  void SharedControlVelocityController::load_parameters()
  {
    // Flags
    declare_and_get_parameters("enable_shared_control_assistance",
                               enable_shared_control_assistance_, true);
    declare_and_get_parameters("enable_velocity_saturation", enable_velocity_saturation_, true);
    declare_and_get_parameters("enable_debug_publish", enable_debug_publish_, false);

    // Filters & Saturation
    declare_and_get_parameters("initial_filter_cutoff_frequency", initial_filter_cutoff_frequency_,
                               0.8);
    declare_and_get_parameters("final_filter_cutoff_frequency", final_filter_cutoff_frequency_,
                               5.0);
    declare_and_get_parameters("max_linear_delta", max_linear_delta_, 0.007);
    declare_and_get_parameters("max_angular_delta", max_angular_delta_, 0.014);

    // Shared Control math
    declare_and_get_parameters("gamma", gamma_, 3.0);
    declare_and_get_parameters("alpha_conf", alpha_conf_, 5.0);
    declare_and_get_parameters("r1", r1_, 0.08);
    declare_and_get_parameters("r2", r2_, 0.040);
    declare_and_get_parameters("v_j_max", v_j_max_, 0.04);

    // Degrees to Radians conversions
    double theta_l_deg, t1_deg, t2_deg, eps_r_deg;
    declare_and_get_parameters("theta_l_deg", theta_l_deg, 10.0);
    declare_and_get_parameters("theta1_deg", t1_deg, 8.0);
    declare_and_get_parameters("theta2_deg", t2_deg, 4.0);
    declare_and_get_parameters("eps_r_reach_deg", eps_r_deg, 5.0);

    theta_l_ = theta_l_deg * M_PI / 180.0;
    theta1_ = t1_deg * M_PI / 180.0;
    theta2_ = t2_deg * M_PI / 180.0;
    eps_r_reach_rad_ = eps_r_deg * M_PI / 180.0;

    declare_and_get_parameters("eps_t_reach_m", eps_t_reach_, 0.01);
    declare_and_get_parameters("reach_dwell_ms", dwell_ms_, 300);
  }

  bool SharedControlVelocityController::configure_goals()
  {
    auto node = get_node();
    //202606
    //std::vector<double> raw_poses;
    //std::vector<std::string> goals_label;
    //declare_and_get_parameters("goals_poses", raw_poses, {0.605, 0.220, 0.278, 0.0, 0.0, 0.0, 1.0});
    //declare_and_get_parameters("goals_labels", goals_label, {"G1"});

    //if (raw_poses.size() % 7 != 0)
    //{
      //RCLCPP_FATAL(node->get_logger(), "goals_poses must be 7*N. Got %zu", raw_poses.size());
      //return false;
    //}
    //param_goals_.clear();

    std::vector<double> raw_poses;
    std::vector<std::string> goals_label;

    //declare_and_get_parameters("goals_poses", raw_poses, std::vector<double>{});
    //declare_and_get_parameters("goals_labels", goals_label, std::vector<std::string>{});
    if (node->has_parameter("goals_poses"))
    {
      node->get_parameter("goals_poses", raw_poses);
    }
    else
    {
      raw_poses.clear();
    }

    if (node->has_parameter("goals_labels"))
    {
      node->get_parameter("goals_labels", goals_label);
    }
    else
    {
      goals_label.clear();
    }
    param_goals_.clear();
    if (raw_poses.empty())
    {
      RCLCPP_WARN(node->get_logger(),
                  "No goals loaded from parameters. Set goals_poses and goals_labels to load goals.");
      return true; // Not an error, just no goals to load
    }
    if (raw_poses.size() % 7 != 0)
    {
      RCLCPP_FATAL(node->get_logger(), "goals_poses must be 7*N. Got %zu", raw_poses.size());
      return false;
    }

    
    for (size_t i = 0; i < raw_poses.size() / 7; ++i)
    {
      size_t idx = i * 7;
      Goal g;
      g.x = Eigen::Vector3d(raw_poses[idx], raw_poses[idx + 1], raw_poses[idx + 2]);
      g.q = Eigen::Quaterniond(raw_poses[idx + 6], raw_poses[idx + 3], raw_poses[idx + 4],
                               raw_poses[idx + 5]);
      g.q.normalize();
      g.c = 0.0; // Initial confidence

      const std::string label = (i < goals_label.size() && !goals_label[i].empty()) ? goals_label[i]:"static_goal_" + std::to_string(i+1);
      //param_goals_[goals_label[i]] = g;
      param_goals_[label] = g;

    }

    RCLCPP_INFO(node->get_logger(), "Loaded %zu goals into map.", param_goals_.size());
    return true;
  }

  void SharedControlVelocityController::setup_publishers()
  {
    auto node = get_node();

    latest_twist_pub_ =
        node->create_publisher<geometry_msgs::msg::Twist>("/debug/latest_twist", 10);
    initial_filtered_linear_velocity_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/initial_filtered_linear_velocity", 10);
    initial_filtered_angular_velocity_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/initial_filtered_angular_velocity", 10);
    assisted_linear_velocity_pub_ =
        node->create_publisher<geometry_msgs::msg::Vector3>("/debug/assisted_linear_velocity", 10);
    assisted_angular_velocity_pub_ =
        node->create_publisher<geometry_msgs::msg::Vector3>("/debug/assisted_angular_velocity", 10);
    filtered_linear_velocity_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/final_filtered_linear_velocity", 10);
    filtered_angular_velocity_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/final_filtered_angular_velocity", 10);
    conf_pub_ =
        node->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/goal_confidences", 10);
    soft_goal_pub_ =
        node->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/soft_goal", 10);
    agnostic_goal_pub_ =
        node->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/agnostic_goal", 10);
    beep_pub_ = node->create_publisher<std_msgs::msg::Bool>("/feedback/beep_trigger", 10);
    goal_reached_pub_ = node->create_publisher<std_msgs::msg::String>("/debug/goal_reached", 10);
    mode_pub_ = node->create_publisher<std_msgs::msg::Int32>("/debug/mode", 10);
    shared_control_debug_pub_ = node->create_publisher<visual_servoing::msg::SharedControlDebug>("/shared_control/debug", 10);
  }

  bool SharedControlVelocityController::init_robot_interface()
  {
    auto node = get_node();
    std::string robot_type, base_frame, tool_frame, robot_description;

    declare_and_get_parameters("robot_type", robot_type, std::string("kinova_velocity"));
    declare_and_get_parameters("tool_frame", tool_frame, std::string("end_effector_link"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});

    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type);
    robot_vel_interface_->set_commands_names(command_names_);

    return robot_vel_interface_ &&
           robot_vel_interface_->initKinematics(robot_description, tool_frame);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  SharedControlVelocityController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    load_parameters();

    if (!configure_goals())
    {
      return CallbackReturn::ERROR;
    }

    setup_publishers();
    if (!init_robot_interface())
    {
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  void SharedControlVelocityController::reset_controller_state()
  {
    previous_initial_filtered_linear_velocity_.setZero();
    previous_initial_filtered_angular_velocity_.setZero();
    previous_cartesian_linear_velocity_.setZero();
    previous_cartesian_angular_velocity_.setZero();
    previous_filtered_linear_velocity_.setZero();
    previous_filtered_angular_velocity_.setZero();

    lpf_initialized_ = false;
    latest_twist_ = geometry_msgs::msg::Twist{};
    last_reached_goal_id_ = "";
    beeped_for_this_visit_ = false;
  }

  void SharedControlVelocityController::update_current_pose()
  {
    auto temp_pose = robot_vel_interface_->getCurrentEndEffectorPose();
    current_position_ = temp_pose.translation;
    current_orientation_ = temp_pose.quaternion;
  }

  void SharedControlVelocityController::activate_goals()
  {
    active_goals_.clear();

    Goal agnostic;
    agnostic.x = current_position_;
    agnostic.q = current_orientation_;
    agnostic.c = 1.0;
    active_goals_["G0"] = agnostic;

    // 2. Load the STATIC goals from the cache
    for (const auto &g : param_goals_)
    {
      active_goals_.insert({g.first, g.second});
    }

    RCLCPP_INFO(get_node()->get_logger(), "Activated with %zu user goals + G0 Agnostic.",
                active_goals_.size() - 1);
  }

  CallbackReturn SharedControlVelocityController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    robot_vel_interface_->assign_loaned_state(state_interfaces_);

    reset_controller_state();
    update_current_pose();

    activate_goals();

    if (mode_pub_)
    {
      std_msgs::msg::Int32 mode;
      mode.data = static_cast<int>(mode_);
      mode_pub_->publish(mode);
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SharedControlVelocityController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    return CallbackReturn::SUCCESS;
  }

  /* -- Update function -- */
  controller_interface::return_type SharedControlVelocityController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    typedef extender_msgs::msg::TeleopCommand Mode;

    robot_vel_interface_->syncState();
    
    /* -- Update state variables --*/
    // Get time
    const rclcpp::Time now = get_node()->now();
    // Get controller period
    const double dt_sec = period.seconds();
    // Get current EE pose.
    robot_interfaces::CartesianPosition temp_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();
    current_position_ = temp_pose.translation;
    current_orientation_ = temp_pose.quaternion;
    // Sync agnostic goal: G0 pose = current EE pose
    if (!active_goals_.empty())
    {
      active_goals_["G0"].x = current_position_;
      active_goals_["G0"].q = current_orientation_;
    }

    /* -- Raw velocities from fr3_teleop twist -- */
    const Eigen::Vector3d raw_linear_velocity(latest_twist_.linear.x, latest_twist_.linear.y,
                                              latest_twist_.linear.z);

    const Eigen::Vector3d raw_angular_velocity(latest_twist_.angular.x, latest_twist_.angular.y,
                                               latest_twist_.angular.z);

    /* -- Apply a low-pass filter to the latest twist command -- */
    // Compute alpha from desired cutoff frequency: alpha = T/(T+tau)
    double alpha_initial_filter = 1.0; // default: passthrough if fc <= 0

   /*  if (initial_filter_cutoff_frequency_ > 0.0)
    {
      const double tau = 1.0 / (2.0 * M_PI * initial_filter_cutoff_frequency_);
      if (dt_sec > 0.0)
      {
        alpha_initial_filter = std::clamp(dt_sec / (dt_sec + tau), 1e-9, 1.0 - 1e-9);
      }
    } // else alpha stays 1 so there's no filtering
 */
    if (!lpf_initialized_)
    {
      previous_initial_filtered_linear_velocity_ = raw_linear_velocity;
      previous_initial_filtered_angular_velocity_ = raw_angular_velocity;
      previous_cartesian_linear_velocity_ = Eigen::Vector3d::Zero();
      previous_cartesian_angular_velocity_ = Eigen::Vector3d::Zero();
      lpf_initialized_ = true;
    }

    const Eigen::Vector3d initial_filtered_linear_velocity = raw_linear_velocity;//applyLowPassFilterVector(
        //raw_linear_velocity, previous_initial_filtered_linear_velocity_, alpha_initial_filter);

    const Eigen::Vector3d initial_filtered_angular_velocity = raw_angular_velocity;//applyLowPassFilterVector(
        //raw_angular_velocity, previous_initial_filtered_angular_velocity_, alpha_initial_filter);

    // Update memory
/*     previous_initial_filtered_linear_velocity_ = initial_filtered_linear_velocity;
    previous_initial_filtered_angular_velocity_ = initial_filtered_angular_velocity;
 */
    Eigen::Vector3d initial_filtered_angular_velocity_base = initial_filtered_angular_velocity;
    if (input_twist_frame_ == "ee")
    {
      // Rotation from End-Effector frame to Base frame
      const Eigen::Matrix3d R_BE = current_orientation_.toRotationMatrix();
      // Convert joystick angular velocity (EE frame) to Base frame
      initial_filtered_angular_velocity_base = R_BE * initial_filtered_angular_velocity;
    }
    // else: base frame -> do nothing

    /* -- Update Goals confidences only in MODE T (Translation_Rotation) -- */
    if (mode_ == Mode::TRANSLATION_ROTATION)
    {
      updateConfidences(dt_sec, initial_filtered_linear_velocity);
    }

    /* -- Compute goal selection based on confidence index -- */
    Goal soft_goal = computeSoftGoal();

    /* -- Check goals -- */
    // Feedback check of known goals
    checkKnownGoalsAndBeep(now);

    /* -- Shared-control -- */
    Eigen::Vector3d cartesian_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d cartesian_angular_velocity = Eigen::Vector3d::Zero();

    if (enable_shared_control_assistance_)
    {
      // -- Shared control assistance -- //
      switch (mode_)
      {
      case Mode::TRANSLATION_ROTATION: { // MODE T

        auto [shaped_velocity, shaped_omega] = applySharedControlModeT(
            soft_goal, current_position_, current_orientation_, initial_filtered_linear_velocity);

        cartesian_linear_velocity = shaped_velocity;
        cartesian_angular_velocity = shaped_omega;
        break;
      }
      case Mode::ROTATION: { // MODE W
        auto [shaped_velocity, shaped_omega] =
            applySharedControlModeW(soft_goal, current_position_, current_orientation_,
                                    initial_filtered_angular_velocity_base);

        cartesian_linear_velocity = shaped_velocity;
        cartesian_angular_velocity = shaped_omega;
        break;
      }
      default:
        cartesian_linear_velocity.setZero();
        cartesian_angular_velocity.setZero();
        break;
      }
    }
    else
    {
      // --- Baseline: agnostic control mode ---
      switch (mode_)
      {
      case Mode::TRANSLATION_ROTATION: {
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        // Compute baseline angular velocity to keep wrist aligned
        cartesian_angular_velocity =
            computeBaselineAngularVelocity(current_position_, cartesian_linear_velocity);
        break;
      }
      case Mode::ROTATION: {
        cartesian_linear_velocity.setZero();
        // User commands rotation in the end-effector frame, command is sent to robot in base frame
        cartesian_angular_velocity = initial_filtered_angular_velocity_base;
        break;
      }
      case Mode::TRANSLATION: {
        // Used for debug in advanced mode: allow_full_mode = true in fr3_teleop
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        cartesian_angular_velocity.setZero();
        break;
      }
      // TODO: legacy, to remove
      case Mode::BOTH: { // not like Mode W, just to use spacenav as
                         // full 6D joystick
        // Used for debug in advanced mode: allow_full_mode = true in fr3_teleop
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        cartesian_angular_velocity = initial_filtered_angular_velocity;
        break;
      }
      default:
        cartesian_linear_velocity.setZero();
        cartesian_angular_velocity.setZero();
        break;
      }
    }

    /* -- Cartesian velocity saturation -- */
    // Limit the velocity change between cycles to avoid discontinuties.
    Eigen::Vector3d saturated_cartesian_linear_velocity = cartesian_linear_velocity;
    Eigen::Vector3d saturated_cartesian_angular_velocity = cartesian_angular_velocity;

    if (enable_velocity_saturation_)
    {
      std::tie(saturated_cartesian_linear_velocity, saturated_cartesian_angular_velocity) =
          applyVelocitySaturation(cartesian_linear_velocity, previous_cartesian_linear_velocity_,
                                  max_linear_delta_, cartesian_angular_velocity,
                                  previous_cartesian_angular_velocity_, max_angular_delta_);

      // Update memory
      previous_cartesian_linear_velocity_ = saturated_cartesian_linear_velocity;
      previous_cartesian_angular_velocity_ = saturated_cartesian_angular_velocity;
    }
    else
    { // Avoid spikes when turning saturation on
      previous_cartesian_linear_velocity_ = cartesian_linear_velocity;
      previous_cartesian_angular_velocity_ = cartesian_angular_velocity;
    }
    // -----

    /* -- Post-saturation low-pass filter (5 Hz default) -- */
    double alpha_final_filter = 1.0; // default: passthrough if fc <= 0
    if (final_filter_cutoff_frequency_ > 0.0)
    {
      const double tau_final_filter = 1.0 / (2.0 * M_PI * final_filter_cutoff_frequency_);
      if (dt_sec > 0.0)
      {
        alpha_final_filter = std::clamp(dt_sec / (dt_sec + tau_final_filter), 1e-9, 1.0 - 1e-9);
      }
    }

    const Eigen::Vector3d filtered_linear_velocity = cartesian_linear_velocity;
        /*applyLowPassFilterVector(saturated_cartesian_linear_velocity,
                                 previous_filtered_linear_velocity_, alpha_final_filter);*/

    const Eigen::Vector3d filtered_angular_velocity = cartesian_angular_velocity;
        /*applyLowPassFilterVector(saturated_cartesian_angular_velocity,
                                 previous_filtered_angular_velocity_, alpha_final_filter);*/
    
    
    // Update memory
    previous_filtered_linear_velocity_ = filtered_linear_velocity;
    previous_filtered_angular_velocity_ = filtered_angular_velocity;
    /*previous_filtered_linear_velocity_ = filtered_linear_velocity;
    previous_filtered_angular_velocity_ = filtered_angular_velocity;*/

    /* -- Publish data -- */
    if (enable_debug_publish_)
    {
      publishDebugData(latest_twist_, initial_filtered_linear_velocity,
                       initial_filtered_angular_velocity, cartesian_linear_velocity,
                       cartesian_angular_velocity, filtered_linear_velocity,
                       filtered_angular_velocity, soft_goal);
    }

    robot_interfaces::CartesianVelocity vel_cmd;
    vel_cmd.linear = filtered_linear_velocity;
    vel_cmd.angular = filtered_angular_velocity;
    /* -- Send cartesian velocity commands using Franka Cartesian Velocity interface -- */
    if (robot_vel_interface_->setCommand(vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }

  } // update()

  void SharedControlVelocityController::teleopCmdCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    latest_twist_ = msg->twist;
    mode_ = msg->mode;

    // Always publish current mode (either new or unchanged)
    if (mode_pub_)
    {
      std_msgs::msg::Int32 mode_msg;
      mode_msg.data = static_cast<int>(mode_);
      mode_pub_->publish(mode_msg);
    }
  }

  void SharedControlVelocityController::goalCallback(
      const extender_msgs::msg::SharedControlGoalArray::SharedPtr msg)
  {
    if (msg->goal_array.empty())
    {
      return;
    }

    for (const auto &g : msg->goal_array)
    {
      if (active_goals_.find(std::to_string(g.id)) != active_goals_.end())
      {
        // Update existing goal
        active_goals_[std::to_string(g.id)].x =
            Eigen::Vector3d(g.goal_pose.position.x, g.goal_pose.position.y, g.goal_pose.position.z);
        active_goals_[std::to_string(g.id)].q =
            Eigen::Quaterniond(g.goal_pose.orientation.w, g.goal_pose.orientation.x,
                               g.goal_pose.orientation.y, g.goal_pose.orientation.z);
        active_goals_[std::to_string(g.id)].q.normalize();
        continue;
      }
      else
      {
        Goal new_goal;
        new_goal.x =
            Eigen::Vector3d(g.goal_pose.position.x, g.goal_pose.position.y, g.goal_pose.position.z);
        new_goal.q = Eigen::Quaterniond(g.goal_pose.orientation.w, g.goal_pose.orientation.x,
                                        g.goal_pose.orientation.y, g.goal_pose.orientation.z);
        new_goal.q.normalize();
        new_goal.c = 0.0; // Initial confidence

        active_goals_[std::to_string(g.id)] = new_goal;
      }
    }
  }

  std::pair<double, Eigen::Vector3d> SharedControlVelocityController::computeDistanceAndDirection(
      const Eigen::Vector3d &goal_position, const Eigen::Vector3d &current_position) const
  {
    const Eigen::Vector3d direction = goal_position - current_position;
    const double distance = direction.norm();

    Eigen::Vector3d unit_axis_u = Eigen::Vector3d::UnitX();
    if (distance > 1e-9)
    {
      unit_axis_u = direction / distance;
    }

    return {distance, unit_axis_u};
  }

  auto SharedControlVelocityController::computeRotationError(
      const Eigen::Quaterniond &q_goal, const Eigen::Quaterniond &q_current) const -> RotationError
  {
    // Relative rotation (from current to goal)
    Eigen::Quaterniond q_err = q_goal * q_current.conjugate();

    // Shortest path: ensure w >= 0 (q and -q represent same rotation)
    if (q_err.w() < 0.0)
      q_err.coeffs() *= -1.0;

    // Extract angle and axis using Eigen
    Eigen::AngleAxisd angle_axis(q_err);
    const double theta = angle_axis.angle();
    const Eigen::Vector3d axis = angle_axis.axis();

    // SO(3) logarithmic map: e_R = θ * ŵ
    const Eigen::Vector3d rotation_vector = theta * axis;

    return RotationError{theta, axis, rotation_vector};
  }

  Eigen::Vector3d SharedControlVelocityController::applyLowPassFilterVector(
      const Eigen::Vector3d &input, const Eigen::Vector3d &previous, double alpha)
  {
    return alpha * input + (1.0 - alpha) * previous;
  }

  void SharedControlVelocityController::updateConfidences(double dt, const Eigen::Vector3d &vE)
  {
    const double vJmax = v_j_max_;              // measure once
    const double z_scale = 0.2;                 // divide z sensitivity by 5
    const double r_freeze = std::min(r1_, r2_); // define a freeze sphere as the smallest one

    // --- Unit-speed and v1 threshold ---
    const double speed_scale = std::min(1.0, vE.norm() / vJmax); // ∈ [0,1]
    const double v1 = 0.20;                                      // 20% threshold

    // --- θ-normalization to decouple α_conf and θ_l ---
    const double cos_th = std::cos(theta_l_);
    double denom_theta = 1.0 - cos_th;

    if (denom_theta <= 1e-6)
    {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[updateConfidences] theta_l too small (θ=%.6f rad). "
                  "Clamping 1 - cos(θ) to 1e-6 for normalization.",
                  theta_l_);
      denom_theta = 1e-6;
    }

    // --- Step 1: Freeze detection ---
    bool inside_any_r_freeze = false;
    for (const auto &[id, goal] : active_goals_)
    {
      if (id == "G0")
        continue;

      const auto distance_direction = computeDistanceAndDirection(goal.x, current_position_);
      const double epsilon_d_i = distance_direction.first;
      if (epsilon_d_i < r_freeze)
      {
        inside_any_r_freeze = true;
        break; // no need to check further
      }
    }

    // --- Step 2: Confidence update ---
    if (!inside_any_r_freeze && speed_scale >= v1)
    {
      // Outside all r_freeze spheres: integrate confidences for all real goals
      for (auto &[id, goal] : active_goals_)
      {
        if (id == "G0")
          continue;

        const auto [epsilon_d, unit_axis] = computeDistanceAndDirection(goal.x, current_position_);

        // Note: using variables names close to equations notation for clarity.
        // Weighted direction
        Eigen::Vector3d u_W = unit_axis; // weighted unit axis
        u_W.z() *= z_scale;              // scale in z
        const double norm_Wu = u_W.norm();

        // Weighted velocity
        Eigen::Vector3d W_v = vE; // weighted velocity
        W_v.z() *= z_scale;       // scale in z
        const double norm_Wv = W_v.norm();

        // Directional cosine in weighted space
        double cos_phi_w = 0.0; // cosine of the angle in the weigthed space
        if (norm_Wv > 0.0 && norm_Wu > 0.0)
        { // to avoid division by zero
          cos_phi_w = u_W.dot(W_v) / (norm_Wu * norm_Wv);
          cos_phi_w = std::clamp(cos_phi_w, -1.0, 1.0); // to protect against numerical overshoots
        }

        // θ-normalized cone term
        const double normalized_cone = (cos_phi_w - cos_th) / denom_theta;

        // Final confidence derivative
        // Eq. (8): cdot = α_conf * (‖v‖/vJmax) * (cos_phi_w − cos θ_l) / (1 − cos θ_l)
        const double cdot = alpha_conf_ * speed_scale * normalized_cone;

        goal.c = std::clamp(goal.c + cdot * dt, 0.0, 1.0); // cGi[0,1]
      }
    }

    // Agnostic confidence
    double sum = 0.0;
    for (const auto &[id, goal] : active_goals_)
    {
      if (id == "G0")
        continue;
      sum += goal.c;
    }
    active_goals_["G0"].c = std::max(0.0, 1.0 - sum);
  }

  Goal SharedControlVelocityController::computeSoftGoal() const
  {
    double confidence_sum = 0.0;
    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d xG = Eigen::Vector3d::Zero();
    Eigen::Matrix4d markley_matrixQ = Eigen::Matrix4d::Zero(); // Markley accumulator

    for (const auto &[id, goal] : active_goals_)
    {
      if (goal.c <= 0.0)
        continue;

      // Translation accumulator
      weighted_sum += goal.c * goal.x;
      confidence_sum += goal.c;

      // Rotation accumulator
      const Eigen::Vector4d qv(goal.q.w(), goal.q.x(), goal.q.y(), goal.q.z());
      markley_matrixQ += goal.c * (qv * qv.transpose());
    }

    // Weigthed average Translation
    if (confidence_sum > 0.0)
    {
      xG = weighted_sum / confidence_sum;
    } // else: keep zero; shouldn't happen because G0 has c=1

    // SelfAdjointEigenSolver sorts eigenvalues ascending: last col is largest eigenvector
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(markley_matrixQ);
    if (eigen_solver.info() != Eigen::Success)
    {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "[computeSoftGoal] Eigen decomposition failed (info=%d).",
                   static_cast<int>(eigen_solver.info()));
      std::terminate();
    }

    // Weighted average Rotation
    Eigen::Vector4d average_quaternion =
        eigen_solver.eigenvectors().col(3); // eigenvector with largest eigenvalue
    Eigen::Quaterniond rG(average_quaternion(0), average_quaternion(1), average_quaternion(2),
                          average_quaternion(3));
    rG.normalize(); // ensure unit quaternion

    return {xG, rG, 1.0};
  }

  Eigen::Vector3d SharedControlVelocityController::computeBaselineAngularVelocity(
      const Eigen::Vector3d &current_position, const Eigen::Vector3d &linear_velocity) const
  {
    const double x_E = current_position.x();
    const double y_E = current_position.y();
    const double v_x = linear_velocity.x();
    const double v_y = linear_velocity.y();
    const double denom = x_E * x_E + y_E * y_E;

    double omega_z_baseline = 0.0;
    if (denom > 0.0)
    {
      omega_z_baseline = (x_E * v_y - y_E * v_x) / denom;
    }

    return Eigen::Vector3d(0.0, 0.0, omega_z_baseline); // ω_{E,u,T}
  }

  double SharedControlVelocityController::sigmaD(double d) const
  {
    const double r_near = std::min(r1_, r2_);
    const double r_far = std::max(r1_, r2_);
    const double span = r_far - r_near;

    // Degenerate: r1_ == r2_ : step gate at r_near
    if (span == 0.0)
    {
      static bool logged_once = false;
      if (!logged_once)
      {
        RCLCPP_INFO(get_node()->get_logger(),
                    "[sigmaD] Degenerate radii: r1 == r2 == %.3f m. "
                    "Using step gate at r=%.3f (σ_d=0 for d<=r, 1 for d>r).",
                    r1_, r_near);
        logged_once = true;
      }
      return (d > r_near) ? 1.0 : 0.0;
    }

    if (d <= r_near)
      return 0.0;
    if (d >= r_far)
      return 1.0;

    return (d - r_near) / span; // linear ramp for r_near < d < r_far
  }

  double SharedControlVelocityController::sigmaR(double epsilon_r) const
  {
    const double denominator = theta1_ - theta2_;
    if (denominator <= 0.0)
    {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[sigmaR] Invalid θ gate: theta1=%.3f ≤ theta2=%.3f. Forcing σ_r=0.", theta1_,
                  theta2_);
      return 0.0;
    }

    if (epsilon_r <= theta2_)
      return 0.0;
    if (epsilon_r >= theta1_)
      return 1.0;

    return (epsilon_r - theta2_) / denominator; // linear ramp
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applySharedControlModeT(const Goal &soft_goal, const Eigen::Vector3d &current_position,
                              const Eigen::Quaterniond &current_orientation,
                              const Eigen::Vector3d &initial_filtered_linear_velocity)
  {
    // --- Distance & direction to soft goal ---
    const auto [epsilon_d, unit_axis_u] =
        computeDistanceAndDirection(soft_goal.x, current_position);

    // --- σ_d(ε_d) gate ---
    const double sigma = sigmaD(epsilon_d);

    // ------------------------------------------------------------
    // Eq. (5) — Linear assistance: scales the goal-aligned component of v_E by gamma,
    // blending smoothly from user-only (σ_d=0) to full assistance (σ_d=1).
    // The orthogonal component of the velocity is preserved explicitly so that only v_prallel is
    // scaled by gamma (not 1 + gamma).
    // ------------------------------------------------------------
    const Eigen::Vector3d v_parallel =
        initial_filtered_linear_velocity.dot(unit_axis_u) * unit_axis_u;
    const Eigen::Vector3d linear_velocity_assisted =
        initial_filtered_linear_velocity + sigma * (gamma_ - 1.0) * v_parallel;

    // --- Baseline angular command ω_{E,u,T} (agnostic mode T) ---
    const Eigen::Vector3d omega_baseline =
        computeBaselineAngularVelocity(current_position, linear_velocity_assisted);

    // ------------------------------------------------------------
    // Orientation error: log(q_G q_E^{-1}) = ε_r * ŵ
    // (angle ε_r and unit axis ŵ)
    // ------------------------------------------------------------
    //202606
    //const RotationError rotation_error = computeRotationError(soft_goal.q, current_orientation);
    //const double epsilon_r = rotation_error.angle;
    //const Eigen::Vector3d unit_axis_w =
        //(epsilon_r > 0.0) ? rotation_error.axis : Eigen::Vector3d::Zero();
    // ------------------------------------------------------------
    // Z-axis-only alignment:
    // Align camera/tool Z axis with goal/tag Z axis.
    // The robot does not try to match the full tag quaternion orientation
    // ------------------------------------------------------------
    // Local Z axis of current end-effector/camera, expressed in base frame
    //const Eigen::Vector3d z_current = current_orientation * Eigen::Vector3d::UnitZ();
    // Local X axis of current end-effector/camera, expressed in base frame
    //const Eigen::Vector3d z_current = -current_orientation * Eigen::Vector3d::UnitZ();
    // Local Z axis of goal/tag, expressed in base frame
    //const Eigen::Vector3d z_goal = soft_goal.q * Eigen::Vector3d::UnitZ();
    // Angle between current and goal Z axes
    //const double epsilon_r = std::acos(std::clamp(z_current.dot(z_goal), -1.0, 1.0));
    // Unit axis of rotation from current to goal
    //Eigen::Vector3d unit_axis_w = z_current.cross(z_goal);
    //if (unit_axis_w.norm() > 1e-6)
    //{
    //  unit_axis_w.normalize();
    //}
    //else
    //{
     // unit_axis_w = Eigen::Vector3d::Zero(); // if current and goal Z are aligned, no preferred rotation axis
    //}
    const Eigen::Vector3d R_BE = current_orientation_.toRotationMatrix().col(2); // current end-effector Z axis in base frame
    const Eigen::Vector3d R_GB = soft_goal.q.toRotationMatrix().col(2); // goal Z axis in base frame
    const double epsilon_r = std::acos(std::clamp(R_BE.dot(R_GB), -1.0, 1.0));
    RCLCPP_INFO(get_node()->get_logger(), "%lf", epsilon_r);
    Eigen::Vector3d unit_axis_w = R_BE.cross(R_GB);
    if (unit_axis_w.norm() > 1e-6)
    {
      unit_axis_w.normalize();
    }
    else
    {
      unit_axis_w = Eigen::Vector3d::Zero(); // if current and goal Z are aligned, no preferred rotation axis
    }


    // ------------------------------------------------------------
    // Eq. (5) — Angular shaping:
    //   ω_E = (1 - σ_d) * ω_{E,u,T} + σ_d * ( ||v_E|| * (ε_r / ε_d - r2) * ŵ )
    // Edge cases:
    //   - If ed→0 ⇒ σ_d=0 ⇒ ω_E = ω_{E,u,T}
    // ------------------------------------------------------------
    Eigen::Vector3d omega_assisted = omega_baseline;

    if (sigma > 0.0)
    {
      if (epsilon_d <= 0.0)
      {
        // Degenerate case: goal reached in translation ⇒ σ=0 should already hold,
        // but if we get here numerically, keep baseline and inform.
        RCLCPP_INFO(get_node()->get_logger(),
                    "[applySharedControlModeT] Degenerate case: ed ≈ 0 ⇒ σ_d=0. Using baseline ω.");
      }
      else
      {
        const double gain = linear_velocity_assisted.norm() *
                            (epsilon_r / (epsilon_d - r2_)); // ||v_E|| * (ε_r / (ε_d - r2)) [rad/s]
        omega_assisted = (1.0 - sigma) * omega_baseline + sigma * gain * unit_axis_w;
        ;
      }
    }

    return {linear_velocity_assisted, omega_assisted};
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applySharedControlModeW(const Goal &soft_goal, const Eigen::Vector3d &current_position,
                              const Eigen::Quaterniond &current_orientation,
                              const Eigen::Vector3d &initial_filtered_angular_velocity_base)
  {
    // ------------------------------------------------------------
    // Eq. (16) (17) Orientation error: log(q_G q_E^{-1}) = ε_r * ŵ
    // (angle ε_r and unit axis ŵ)
    // ------------------------------------------------------------
    //const RotationError rotation_error = computeRotationError(soft_goal.q, current_orientation);
    //const double epsilon_r = rotation_error.angle;
    //const Eigen::Vector3d unit_axis_w =
        //(epsilon_r > 0.0) ? rotation_error.axis : Eigen::Vector3d::Zero();
    // Axis-only alignment for rotation mode:
    // align robot/tool local X axis with goal/tag local Z axis.
    // This avoids matching the full AprilTag quaternion.
    const Eigen::Vector3d approach_current = current_orientation * Eigen::Vector3d::UnitX();

    const Eigen::Vector3d z_goal = soft_goal.q * Eigen::Vector3d::UnitZ();

    const double epsilon_r = std::acos(std::clamp(approach_current.dot(z_goal), -1.0, 1.0));

    Eigen::Vector3d unit_axis_w = approach_current.cross(z_goal);

    if (unit_axis_w.norm() > 1e-6)
    {
      unit_axis_w.normalize();
    }
    else
    {
      unit_axis_w = Eigen::Vector3d::Zero();
    }
        

    // --- Eq. (18) σ_r(ε_r) gate ---
    const double sigma_r = sigmaR(epsilon_r);

    // --- Eq. (19): amplify ω along ŵ ---
    Eigen::Vector3d omega_assisted = initial_filtered_angular_velocity_base; // ω_{E,u,W}
    if (epsilon_r > 0.0)
    {
      const Eigen::Vector3d omega_parallel =
          (initial_filtered_angular_velocity_base.dot(unit_axis_w)) * unit_axis_w;
      omega_assisted += gamma_ * sigma_r * omega_parallel;
    }

    // --- Eq. (20): reorientation-only translation ---
    const Eigen::Vector3d position_delta = (soft_goal.x - current_position);
    const Eigen::Vector3d linear_velocity_assisted = position_delta.cross(omega_assisted);

    return {linear_velocity_assisted, omega_assisted};
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applyVelocitySaturation(const Eigen::Vector3d &current_linear,
                              const Eigen::Vector3d &previous_linear, const double max_linear_delta,
                              const Eigen::Vector3d &current_angular,
                              const Eigen::Vector3d &previous_angular,
                              const double max_angular_delta)
  {
    Eigen::Vector3d saturated_linear = previous_linear;
    Eigen::Vector3d saturated_angular = previous_angular;

    for (int i = 0; i < 3; ++i)
    {
      const double delta_linear =
          std::clamp(current_linear[i] - previous_linear[i], -max_linear_delta, max_linear_delta);
      const double delta_angular = std::clamp(current_angular[i] - previous_angular[i],
                                              -max_angular_delta, max_angular_delta);

      saturated_linear[i] += delta_linear;
      saturated_angular[i] += delta_angular;
    }

    return {saturated_linear, saturated_angular};
  }

  void SharedControlVelocityController::publishDebugData(
      const geometry_msgs::msg::Twist &latest_twist,
      const Eigen::Vector3d &initial_filtered_linear_velocity,
      const Eigen::Vector3d &initial_filtered_angular_velocity,
      const Eigen::Vector3d &cartesian_linear_velocity,
      const Eigen::Vector3d &cartesian_angular_velocity,
      const Eigen::Vector3d &filtered_linear_velocity,
      const Eigen::Vector3d &filtered_angular_velocity, const Goal &soft_goal)
  {
    // Early exit if nothing to publish
    if (!latest_twist_pub_ && !initial_filtered_linear_velocity_pub_ &&
        !initial_filtered_angular_velocity_pub_ && !assisted_linear_velocity_pub_ &&
        !assisted_angular_velocity_pub_ && !filtered_linear_velocity_pub_ &&
        !filtered_angular_velocity_pub_ && !soft_goal_pub_ && !agnostic_goal_pub_ && !conf_pub_ && !shared_control_debug_pub_)
    {
      return;
    }

    const auto now = get_node()->now();

    auto to_vec3 = [](const Eigen::Vector3d &v) {
      geometry_msgs::msg::Vector3 m;
      m.x = v.x();
      m.y = v.y();
      m.z = v.z();
      return m;
    };

    // Latest twist (already a Twist)
    if (latest_twist_pub_)
      latest_twist_pub_->publish(latest_twist);

    // Initial filtered velocities
    if (initial_filtered_linear_velocity_pub_)
      initial_filtered_linear_velocity_pub_->publish(to_vec3(initial_filtered_linear_velocity));
    if (initial_filtered_angular_velocity_pub_)
      initial_filtered_angular_velocity_pub_->publish(to_vec3(initial_filtered_angular_velocity));

    // Assisted linear / angular velocities
    if (assisted_linear_velocity_pub_)
      assisted_linear_velocity_pub_->publish(to_vec3(cartesian_linear_velocity));
    if (assisted_angular_velocity_pub_)
      assisted_angular_velocity_pub_->publish(to_vec3(cartesian_angular_velocity));

    // Post-saturation velocities
    if (filtered_linear_velocity_pub_)
      filtered_linear_velocity_pub_->publish(to_vec3(filtered_linear_velocity));
    if (filtered_angular_velocity_pub_)
      filtered_angular_velocity_pub_->publish(to_vec3(filtered_angular_velocity));

    // Soft goal pose
    if (soft_goal_pub_)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = now;
      ps.header.frame_id = "base_link";
      ps.pose.position.x = soft_goal.x.x();
      ps.pose.position.y = soft_goal.x.y();
      ps.pose.position.z = soft_goal.x.z();
      ps.pose.orientation.w = soft_goal.q.w();
      ps.pose.orientation.x = soft_goal.q.x();
      ps.pose.orientation.y = soft_goal.q.y();
      ps.pose.orientation.z = soft_goal.q.z();
      soft_goal_pub_->publish(ps);
    }

    // Goal confidences
    if (conf_pub_)
    {
      std_msgs::msg::Float64MultiArray cmsg;
      cmsg.data.reserve(active_goals_.size());
      for (const auto &g : active_goals_)
        cmsg.data.push_back(g.second.c);
      conf_pub_->publish(cmsg);
    }

    // Agnostic goal (G0)
    if (agnostic_goal_pub_ && active_goals_.size() > 1)
    {
      geometry_msgs::msg::PoseStamped g0;
      g0.header.stamp = now;
      g0.header.frame_id = "base_link";
      g0.pose.position.x = active_goals_["G0"].x.x();
      g0.pose.position.y = active_goals_["G0"].x.y();
      g0.pose.position.z = active_goals_["G0"].x.z();
      g0.pose.orientation.w = active_goals_["G0"].q.w();
      g0.pose.orientation.x = active_goals_["G0"].q.x();
      g0.pose.orientation.y = active_goals_["G0"].q.y();
      g0.pose.orientation.z = active_goals_["G0"].q.z();
      agnostic_goal_pub_->publish(g0);
    }

    if (shared_control_debug_pub_)
    {
      visual_servoing::msg::SharedControlDebug dbg;

      dbg.header.stamp = now;
      dbg.header.frame_id = "base_link";

      dbg.end_effector_pose.header = dbg.header;
      dbg.end_effector_pose.pose.position.x = current_position_.x();
      dbg.end_effector_pose.pose.position.y = current_position_.y();
      dbg.end_effector_pose.pose.position.z = current_position_.z();
      dbg.end_effector_pose.pose.orientation.w = current_orientation_.w();
      dbg.end_effector_pose.pose.orientation.x = current_orientation_.x();
      dbg.end_effector_pose.pose.orientation.y = current_orientation_.y();
      dbg.end_effector_pose.pose.orientation.z = current_orientation_.z();

      dbg.user_linear_velocity.x = initial_filtered_linear_velocity.x();
      dbg.user_linear_velocity.y = initial_filtered_linear_velocity.y();
      dbg.user_linear_velocity.z = initial_filtered_linear_velocity.z();

      dbg.assisted_linear_velocity.x = cartesian_linear_velocity.x();
      dbg.assisted_linear_velocity.y = cartesian_linear_velocity.y();
      dbg.assisted_linear_velocity.z = cartesian_linear_velocity.z();

      dbg.soft_goal_pose.header = dbg.header;
      dbg.soft_goal_pose.pose.position.x = soft_goal.x.x();
      dbg.soft_goal_pose.pose.position.y = soft_goal.x.y();
      dbg.soft_goal_pose.pose.position.z = soft_goal.x.z();
      dbg.soft_goal_pose.pose.orientation.w = soft_goal.q.w();
      dbg.soft_goal_pose.pose.orientation.x = soft_goal.q.x();
      dbg.soft_goal_pose.pose.orientation.y = soft_goal.q.y();
      dbg.soft_goal_pose.pose.orientation.z = soft_goal.q.z();

      const auto g0_it = active_goals_.find("G0");
      if (g0_it != active_goals_.end())
      {
        dbg.agnostic_goal_pose.header = dbg.header;
        dbg.agnostic_goal_pose.pose.position.x = g0_it->second.x.x();
        dbg.agnostic_goal_pose.pose.position.y = g0_it->second.x.y();
        dbg.agnostic_goal_pose.pose.position.z = g0_it->second.x.z();
        dbg.agnostic_goal_pose.pose.orientation.w = g0_it->second.q.w();
        dbg.agnostic_goal_pose.pose.orientation.x = g0_it->second.q.x();
        dbg.agnostic_goal_pose.pose.orientation.y = g0_it->second.q.y();
        dbg.agnostic_goal_pose.pose.orientation.z = g0_it->second.q.z();
      }

      dbg.theta_l_rad = theta_l_;
      dbg.r1 = r1_;
      dbg.r2 = r2_;
      dbg.mode = static_cast<int32_t>(mode_);

      const double user_speed = initial_filtered_linear_velocity.norm();

      for (const auto &[goal_key, goal] : active_goals_)
      {
        if (goal_key == "G0")
        {
          continue;
        }

        visual_servoing::msg::SharedControlGoalDebug goal_dbg;

        goal_dbg.id = -1;

        if (goal_key.rfind("tag_", 0) == 0)
        {
          try
          {
            goal_dbg.id = std::stoi(goal_key.substr(4));
          }
          catch (const std::exception &)
          {
            goal_dbg.id = -1;
          }
        }

        goal_dbg.pose.position.x = goal.x.x();
        goal_dbg.pose.position.y = goal.x.y();
        goal_dbg.pose.position.z = goal.x.z();
        goal_dbg.pose.orientation.w = goal.q.w();
        goal_dbg.pose.orientation.x = goal.q.x();
        goal_dbg.pose.orientation.y = goal.q.y();
        goal_dbg.pose.orientation.z = goal.q.z();

        const Eigen::Vector3d diff = goal.x - current_position_;
        const double distance = diff.norm();

        goal_dbg.confidence = goal.c;
        goal_dbg.distance = distance;

        if (distance > 1e-9 && user_speed > 1e-9)
        {
          const Eigen::Vector3d u_i = diff.normalized();
          const Eigen::Vector3d v = initial_filtered_linear_velocity.normalized();

          double cos_angle = u_i.dot(v);
          cos_angle = std::clamp(cos_angle, -1.0, 1.0);

          goal_dbg.angle_rad = std::acos(cos_angle);
        }
        else
        {
          goal_dbg.angle_rad = 0.0;
        }

        goal_dbg.selected = goal.c > 0.5;

        dbg.goals.push_back(goal_dbg);
      }

      shared_control_debug_pub_->publish(dbg);
    }

  }

  void SharedControlVelocityController::checkKnownGoalsAndBeep(const rclcpp::Time &now)
  {
    // Default: publish 0 (no reach) each cycle unless we actually fire a beep
    if (goal_reached_pub_)
    {
      std_msgs::msg::String flag;
      flag.data = "";
      goal_reached_pub_->publish(flag);
    }

    // Quick exit if no known goals configured
    if (active_goals_.size() <= 1)
    {
      // reset state
      beeped_for_this_visit_ = false;
      return;
    }

    /* -- Find the best candidate inside reach (min distance among those that pass thresholds) -- */
    std::string best_goal_id = "";
    double best_distance = std::numeric_limits<double>::infinity();

    // Strict 6D check: both translation and orientation must be within thresholds
    for (const auto &[id, goal] : active_goals_)
    {
      if (id == "G0")
        continue; // skip agnostic goal

      const double distance = (goal.x - current_position_).norm();
      if (distance > eps_t_reach_)
        continue;

      const double orientation_error = computeRotationError(goal.q, current_orientation_).angle;
      if (orientation_error > eps_r_reach_rad_)
        continue;

      if (distance < best_distance)
      {
        best_distance = distance;
        best_goal_id = id;
      }
    }

    if (best_goal_id != "")
    {
      // Check if we just entered this goal's radius
      if (last_reached_goal_id_ != best_goal_id)
      {
        last_reached_goal_id_ = best_goal_id;
        reached_enter_time_ = now;
        beeped_for_this_visit_ = false;
      }

      // Check if we have dwelled long enough
      if (!beeped_for_this_visit_)
      {
        const double elapsed_ms = (now - reached_enter_time_).nanoseconds() / 1e6;
        if (elapsed_ms >= static_cast<double>(dwell_ms_))
        {
          trigger_reach_feedback(best_goal_id, best_distance);
          beeped_for_this_visit_ = true;
        }
      }
    }
    else
    {
      // Out of range of all goals
      last_reached_goal_id_ = "";
      beeped_for_this_visit_ = false;
    }
  }

  void SharedControlVelocityController::trigger_reach_feedback(const std::string &id,
                                                               double distance)
  {
    if (beep_pub_)
    {
      std_msgs::msg::Bool msg;
      msg.data = true;
      beep_pub_->publish(msg);
    }

    if (goal_reached_pub_)
    {
      std_msgs::msg::String msg;
      msg.data = id;
      goal_reached_pub_->publish(msg);
    }

    RCLCPP_INFO(get_node()->get_logger(), "[REACHED] ID %s - Dist: %.3f m", id.c_str(), distance);
  }

  //202603
  bool SharedControlVelocityController::isSameDetectedGoal(
    const Goal &existing_goal,
    const geometry_msgs::msg::Pose &new_pose,
    double pos_tol,
    double ang_tol_rad) const
  {
    // ---------------------------------------------------------------------------
    // Position comparison
    // ---------------------------------------------------------------------------
    // Convert the newly detected pose position into an Eigen vector and compare it
    // with the existing goal position.
    const Eigen::Vector3d new_position(
        new_pose.position.x,
        new_pose.position.y,
        new_pose.position.z);

    const double position_error = (existing_goal.x - new_position).norm();

    // ---------------------------------------------------------------------------
    // Orientation comparison
    // ---------------------------------------------------------------------------
    // Convert the newly detected pose orientation into an Eigen quaternion.
    Eigen::Quaterniond new_orientation(
        new_pose.orientation.w,
        new_pose.orientation.x,
        new_pose.orientation.y,
        new_pose.orientation.z);

    // Protect against invalid quaternions.
    if (new_orientation.norm() < 1e-9)
    {
      new_orientation = Eigen::Quaterniond::Identity();
    }
    else
    {
      new_orientation.normalize();
    }

    // Compute the angular error between the existing goal orientation and the new
    // detected orientation.
    const double angular_error =
        computeRotationError(existing_goal.q, new_orientation).angle;

    // The detected goal is considered the same if both position and orientation
    // errors are below their thresholds.
    return (position_error <= pos_tol) && (angular_error <= ang_tol_rad);
  }

  //202603
void SharedControlVelocityController::goalsCallback(
    const visual_servoing::msg::DetectedGoalArray::SharedPtr msg)
  {
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Received %zu detected AprilTag goal(s).",
        msg->goals.size());

    // ---------------------------------------------------------------------------
    // No visible AprilTags
    // ---------------------------------------------------------------------------
    // Remove only AprilTag dynamic goals. Keep G0 and parameter/static goals.
    if (msg->goals.empty())
    {
      for (const auto &[tag_id, goal_key] : active_goal_key_by_tag_id_)
      {
        active_goals_.erase(goal_key);
      }

      active_goal_key_by_tag_id_.clear();

      RCLCPP_WARN(
          get_node()->get_logger(),
          "No AprilTag goals received. Removed all active AprilTag goals.");

      return;
    }

    std::unordered_set<int> seen_ids;

    // ---------------------------------------------------------------------------
    // Add or update currently visible AprilTags
    // ---------------------------------------------------------------------------
    for (const auto &detected_goal_msg : msg->goals)
    {
      const int tag_id = detected_goal_msg.id;
      const auto &pose = detected_goal_msg.pose;

      seen_ids.insert(tag_id);

      const std::string goal_key = "tag_" + std::to_string(tag_id);

      // Convert pose to Goal.
      Goal detected_goal;

      detected_goal.x = Eigen::Vector3d(
          pose.position.x,
          pose.position.y,
          pose.position.z);

      Eigen::Quaterniond orientation(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);

      if (orientation.norm() < 1e-9)
      {
        orientation = Eigen::Quaterniond::Identity();
      }
      else
      {
        orientation.normalize();
      }

      detected_goal.q = orientation;

      // -------------------------------------------------------------------------
      // Existing AprilTag goal
      // -------------------------------------------------------------------------
      auto existing_goal_it = active_goals_.find(goal_key);

      if (existing_goal_it != active_goals_.end())
      {
        // Preserve confidence.
        detected_goal.c = existing_goal_it->second.c;

        if (isSameDetectedGoal(existing_goal_it->second, pose))
        {
          // Pose close enough: do not rewrite pose, just keep it.
          RCLCPP_INFO(
              get_node()->get_logger(),
              "Tag ID %d already active. Pose is close. Keeping goal '%s' with confidence %.3f.",
              tag_id,
              goal_key.c_str(),
              existing_goal_it->second.c);
        }
        else
        {
          // Same tag moved significantly: update pose, preserve confidence.
          active_goals_[goal_key] = detected_goal;

          RCLCPP_INFO(
              get_node()->get_logger(),
              "Tag ID %d moved. Updated goal '%s', kept confidence %.3f.",
              tag_id,
              goal_key.c_str(),
              detected_goal.c);
        }
      }
      // -------------------------------------------------------------------------
      // New AprilTag goal
      // -------------------------------------------------------------------------
      else
      {
        detected_goal.c = 0.0;
        active_goals_[goal_key] = detected_goal;

        RCLCPP_INFO(
            get_node()->get_logger(),
            "New tag ID %d added as active goal '%s' with confidence 0.0.",
            tag_id,
            goal_key.c_str());
      }

      active_goal_key_by_tag_id_[tag_id] = goal_key;
    }

    // ---------------------------------------------------------------------------
    // Remove AprilTag goals that disappeared
    // ---------------------------------------------------------------------------
    for (auto it = active_goal_key_by_tag_id_.begin();
        it != active_goal_key_by_tag_id_.end();)
    {
      const int tag_id = it->first;
      const std::string goal_key = it->second;

      if (seen_ids.find(tag_id) == seen_ids.end())
      {
        active_goals_.erase(goal_key);

        RCLCPP_INFO(
            get_node()->get_logger(),
            "Tag ID %d disappeared. Removed active goal '%s'.",
            tag_id,
            goal_key.c_str());

        it = active_goal_key_by_tag_id_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }


} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::SharedControlVelocityController,
                       controller_interface::ControllerInterface)
