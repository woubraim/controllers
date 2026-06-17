#pragma once

#include <unordered_map>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"
#include "extender_msgs/msg/teleop_command.hpp"

#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

#include "visual_servoing/msg/detected_goal_array.hpp"
#include "visual_servoing/msg/shared_control_debug.hpp"

namespace cartesian_velocity_controller
{
  /**
   * @struct Goal
   * @brief Represents a shared control goal with position, orientation, and confidence.
   */
  struct Goal
  {
    Eigen::Vector3d x;    ///< Desired position in Cartesian space.
    Eigen::Quaterniond q; ///< Desired orientation as a quaternion.
    double c;             ///< Confidence value in [0,1] indicating goal reliability.
  };

  /**
   * @struct RotationError
   * @brief Represents the rotational error between two orientations.
   */
  struct RotationError
  {
    double angle;                    ///< Rotation angle ε_r ∈ [0, π] in radians.
    Eigen::Vector3d axis;            ///< Rotation axis ŵ (zero if angle == 0).
    Eigen::Vector3d rotation_vector; ///< Rotation vector e_R = angle * axis (log map), zero if angle == 0.
  };

  /**
   * @class SharedControlVelocityController
   * @brief A ROS2 controller that implements shared control for Cartesian velocity commands with kinematic guides.
   *
   * This controller blends user-provided velocity commands with autonomous assistance based on predefined goals.
   * It supports multiple modes (translation, rotation, combined) and includes features such as:
   * - Shared control assistance using confidence-based goal blending
   * - Velocity saturation and low-pass filtering
   * - Goal-based feedback and dwell detection
   * - Debug publishing for analysis
   *
   * The controller uses kinematic guides to provide assistance in reaching goals while allowing user control.
   */
  class SharedControlVelocityController : public controller_interface::ControllerInterface
  {
  public:
    /**
     * @brief Constructor for SharedControlVelocityController.
     */
    SharedControlVelocityController();

    /**
     * @brief Destructor for SharedControlVelocityController.
     */
    virtual ~SharedControlVelocityController();

    /**
     * @brief Configures the command interfaces required by the controller.
     * @return InterfaceConfiguration specifying the command interfaces.
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief Configures the state interfaces required by the controller.
     * @return InterfaceConfiguration specifying the state interfaces.
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief Main update loop called periodically by the controller manager.
     * @param time Current time.
     * @param period Time since last update.
     * @return Return type indicating success or failure.
     */
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    /**
     * @brief Lifecycle callback for initialization.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

    /**
     * @brief Lifecycle callback for configuration.
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Lifecycle callback for activation.
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Lifecycle callback for deactivation.
     * @param previous_state Previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

  private:
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      auto node = get_node();
      if (!node->has_parameter(name))
      {
        node->declare_parameter(name, default_value);
      }
      variable = node->get_parameter(name).get_value<T>();
    }

    /**
     * @brief Loads all controller parameters from the ROS parameter server.
     */
    void load_parameters();

    /**
     * @brief Configures goals from parameters.
     * @return True if goals are configured successfully, false otherwise.
     */
    bool configure_goals();

    /**
     * @brief Sets up all ROS publishers.
     */
    void setup_publishers();

    /**
     * @brief Initializes the robot interface component.
     * @return True if initialization succeeds, false otherwise.
     */
    bool init_robot_interface();

    /**
     * @brief Resets the controller state to initial values.
     */
    void reset_controller_state();

    /**
     * @brief Updates the current pose of the robot end-effector.
     */
    void update_current_pose();

    /**
     * @brief Activates goals based on current state.
     */
    void activate_goals();

    /**
     * @brief Callback to receive Twist + Mode commands from the teleop node.
     *
     * The incoming message contains:
     *   • msg->twist : geometry_msgs::Twist with user linear/angular velocity commands
     *   • msg->mode  : integer enum corresponding to a TeleopMode (e.g. Translation,
     *                  Rotation, or Translation_Rotation)
     *
     * The callback updates the latest twist command and switches control mode if it
     * differs from the current one. In all cases, the current mode is published on
     * the /debug/mode topic for visualization and logging.
     * @param msg Shared pointer to the TeleopCommand message.
     */
    void teleopCmdCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);

    /**
     * @brief Callback to receive shared control goals.
     * @param msg Shared pointer to the SharedControlGoalArray message.
     */
    void goalCallback(const extender_msgs::msg::SharedControlGoalArray::SharedPtr msg);

    /**
     * @brief Computes the minimal 3D rotation required to align the current end-effector
     * orientation (q_current) with the desired goal orientation (q_goal).
     *
     * The relative rotation quaternion is defined as:
     *     q_err = q_goal * q_current.conjugate()
     * which represents the rotation that transforms the current frame to the goal
     * frame.
     *
     * Because unit quaternions q and -q represent the same physical rotation,
     * the scalar part w is flipped to ensure w ≥ 0 so that the shortest rotation
     * path is always used to avoid discontinuities.
     *
     * The Eigen::AngleAxisd representation is then used to extract:
     *   • angle  θ  = angle_axis.angle()       total rotation angle ε_r ∈ [0, π]
     *   • axis   ŵ  = angle_axis.axis()        unit rotation axis
     *   • e_R = θ * ŵ                          SO(3) logarithmic map (rotation vector)
     *
     * Returns a RotationError struct { angle, axis, rotation_vector }.
     * @param q_goal Target orientation quaternion.
     * @param q_current Current orientation quaternion.
     * @return RotationError struct containing angle, axis, and rotation vector.
     */
    RotationError computeRotationError(const Eigen::Quaterniond &q_goal,
                                       const Eigen::Quaterniond &q_current) const;

    /**
     * @brief Applies a first-order discrete low-pass filter to a 3D input vector.
     *
     * Implements the standard exponential smoothing form:
     *     y[n] = α * x[n] + (1 - α) * y[n-1]
     * where:
     *   • x[n]       current (raw) input vector
     *   • y[n-1]     previously filtered output
     *   • α ∈ [0,1]  filter gain (computed as T / (T + τ))
     *
     * Interpretation:
     *   - α = 1  minimal filtering (fast response)
     *   - α = 0  strong filtering (slow response)
     *   - τ      time constant of the filter (s)
     *   - T      sampling period (s)
     *
     * This filter is applied component-wise on the input Eigen::Vector3d.
     * It is used to smooth measured or commanded Cartesian velocities.
     * @param input Current input vector.
     * @param previous Previous filtered output.
     * @param alpha Filter coefficient (0 < alpha < 1).
     * @return Filtered vector.
     */
    Eigen::Vector3d applyLowPassFilterVector(const Eigen::Vector3d &input,
                                             const Eigen::Vector3d &previous, double alpha);

    /**
     * @brief Computes the Euclidean distance ε_d and unit direction û from the current
     * end-effector position x_E to the goal position x_G.
     *
     * Returns a pair {ε_d, û}, where:
     *   ε_d = ‖x_G − x_E‖
     *   û   = (x_G − x_E) / ε_d   if ε_d > 0
     *       = [1, 0, 0]ᵀ          otherwise (default axis)
     * @param goal_position Goal position.
     * @param current_position Current end-effector position.
     * @return Pair containing distance ε_d and unit direction û.
     */
    std::pair<double, Eigen::Vector3d> computeDistanceAndDirection(
        const Eigen::Vector3d &goal_position, const Eigen::Vector3d &current_position) const;


    /**
     * @brief Computes the baseline angular velocity ω_{E,u,T} used in MODE T
     * to keep the wrist aligned with the mobile (base) frame.
     *
     * Implements the geometric relationship:
     *     ω_z = (x_E * v_y - y_E * v_x) / (x_E² + y_E²)
     *
     * This expression represents the instantaneous angular rate needed
     * to maintain constant wrist orientation relative to the mobile frame
     * during translational motion. When the end-effector moves purely
     * along the base frame axes, ω_z = 0.
     * @param current_position Current end-effector position.
     * @param linear_velocity Current linear velocity.
     * @return Baseline angular velocity vector.
     */
    Eigen::Vector3d computeBaselineAngularVelocity(const Eigen::Vector3d &current_position,
                                                   const Eigen::Vector3d &linear_velocity) const;

    /**
     * @brief Implements the σ_d(d) distance-based blending gate used in Eq. (5):
     *
     *   σ_d(d) = 0                  for d ≤ r_near
     *          = (d - r_near)/span  for r_near < d < r_far
     *          = 1                  for d ≥ r_far
     *
     * where r_near = min(r₁, r₂) and r_far = max(r₁, r₂).
     *
     * This gate allows a smooth transition between pure user control (σ_d=0) and
     * full shared control (σ_d=1) depending on the distance to the target.
     *
     * Degenerate case: if r₁ == r₂, σ_d becomes a binary step at r_near.
     * @param d Distance to goal.
     * @return Gate value between 0 and 1.
     */
    double sigmaD(double d) const;

    /**
     * @brief Implements the σ_r(ε_r) angular blending gate used in Eq. (18)
     * to control how much rotational assistance is applied.
     *
     * Definition:
     *     σ_r(ε_r) = 0                     for ε_r ≤ θ₂
     *               = (ε_r - θ₂)/(θ₁ - θ₂)  for θ₂ < ε_r < θ₁
     *               = 1                     for ε_r ≥ θ₁
     *
     * where:
     *   • ε_r  = current orientation error angle [rad]
     *   • θ₁, θ₂ = angular thresholds defining the transition region
     *               (typically θ₁ > θ₂)
     *
     * The gate increases smoothly from 0 to 1 as the orientation
     * error ε_r grows, allowing gradual blending between user-only
     * control (σ_r = 0) and shared control (σ_r = 1).
     *
     * Degenerate case:
     *   If θ₁ ≤ θ₂, the function logs a warning and returns σ_r = 0.
     * @param epsilon_r Rotational error angle.
     * @return Gate value between 0 and 1.
     */
    double sigmaR(double epsilon_r) const;

    /**
     * @brief Shared-control shaping for MODE T (Translation)
     *
     * Equations:
     *   (5)  v_E = v_{E,u,T} + σ_d · (γ − 1) · (û ûᵀ) · v_{E,u,T}
     *   (15) ω_E = (1 − σ_d) · ω_{E,u,T} + σ_d · (‖v_E‖ · (ε_r / (ε_d − r₂)) · ŵ)
     *
     * Inputs:
     *   - soft_goal.x : goal position x_G
     *   - soft_goal.q : goal orientation q_G
     *   - current_position    : end-effector position x_E (in base frame)
     *   - current_orientation : end-effector orientation q_E (in base frame)
     *   - initial_filtered_linear_velocity : user linear command v_J (filtered in base frame)
     *
     * Output:
     *   - pair { linear_velocity_assisted v_E , omega_assisted ω_E }
     *
     * Notes:
     *   • σ_d(ε_d) gates assistance based on distance to the goal.
     *   • Only the goal-aligned velocity component v_∥ is scaled by γ; v_⊥ is preserved.
     *   • ω_E blends between the baseline wrist alignment ω_{E,u,T} and orientation-driven
     *     assistance proportional to ε_r / (ε_d − r₂).
     * @param soft_goal Current soft goal.
     * @param current_position Current end-effector position.
     * @param current_orientation Current end-effector orientation.
     * @param initial_filtered_linear_velocity Initial filtered linear velocity.
     * @return Pair of linear and angular velocity vectors after shared control.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applySharedControlModeT(
        const Goal &soft_goal, const Eigen::Vector3d &current_position,
        const Eigen::Quaterniond &current_orientation,
        const Eigen::Vector3d &initial_filtered_linear_velocity);

    /**
     * @brief Shared-control shaping for MODE W (Rotation-only)
     *
     * Equations:
     *   (18) σ_r(ε_r) = sat(0,1, (ε_r - θ2) / (θ1 - θ2))
     *   (19) ω_E       = ω_{E,u,W} + γ · σ_r · (ŵ ŵᵀ) · ω_{E,u,W}
     *   (20) v_E       = (x_G - x_E) × ω_E
     *
     * Inputs:
     *   - soft_goal.x : goal position x_G
     *   - soft_goal.q : goal orientation q_G
     *   - current_position    : end-effector position x_E (in base frame)
     *   - current_orientation : end-effector orientation q_E (in base frame)
     *   - initial_filtered_angular_velocity_base : user angular command ω_{E,u,W} (already in base
     *   frame)
     *
     * Output:
     *   - pair { linear_velocity_assisted v_E , omega_assisted ω_E }
     *
     * Notes:
     *   • ε_r and ŵ come from the shortest-path quaternion error log(q_G · q_E^{-1}).
     *   • σ_r gates assistance by orientation error (no help near alignment).
     *   • Linear velocity is the reorientation-only term so the EE rotates about x_G.
     * @param soft_goal Current soft goal.
     * @param current_position Current end-effector position.
     * @param current_orientation Current end-effector orientation.
     * @param initial_filtered_angular_velocity_base Initial filtered angular velocity in base frame.
     * @return Pair of linear and angular velocity vectors after shared control.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applySharedControlModeW(
        const Goal &soft_goal, const Eigen::Vector3d &current_position,
        const Eigen::Quaterniond &current_orientation,
        const Eigen::Vector3d &initial_filtered_angular_velocity_base);

    /**
     * @brief Limits the rate of change of both linear and angular Cartesian velocities
     * between consecutive control cycles to prevent discontinuities or jerks.
     *
     * Implements element-wise saturation on each velocity component:
     *
     *   v_sat[i] = v_prev[i] + clamp(v_curr[i] - v_prev[i],
     *                                -Δv_max, +Δv_max)
     *
     * where:
     *   • v_prev : previous (already sent) velocity vector
     *   • v_curr : newly computed velocity vector
     *   • Δv_max : maximum allowed increment per control step
     *
     * Inputs:
     *   - current_linear / current_angular : current velocity commands
     *   - previous_linear / previous_angular : previous commanded velocities
     *   - max_linear_delta / max_angular_delta : per-axis limits [m/s] / [rad/s]
     *
     * Returns:
     *   pair { saturated_linear, saturated_angular }
     *
     * Notes:
     *   • Prevents acceleration spikes and discontinuities caused by numerical noise
     *     or abrupt joystick inputs.
     * @param current_linear Current linear velocity.
     * @param previous_linear Previous linear velocity.
     * @param max_linear_delta Maximum allowed linear velocity change.
     * @param current_angular Current angular velocity.
     * @param previous_angular Previous angular velocity.
     * @param max_angular_delta Maximum allowed angular velocity change.
     * @return Pair of saturated linear and angular velocities.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applyVelocitySaturation(
        const Eigen::Vector3d &current_linear, const Eigen::Vector3d &previous_linear,
        const double max_linear_delta, const Eigen::Vector3d &current_angular,
        const Eigen::Vector3d &previous_angular, const double max_angular_delta);

    /**
     * @brief Publishes all relevant debug signals for visualization and logging,
     * including raw, guided, and filtered velocities, goal confidences,
     * and poses (soft and agnostic goals) for RViz and PlotJuggler.
     * @param latest_twist Latest twist command.
     * @param initial_filtered_linear_velocity Initial filtered linear velocity.
     * @param initial_filtered_angular_velocity Initial filtered angular velocity.
     * @param cartesian_linear_velocity Cartesian linear velocity after assistance.
     * @param cartesian_angular_velocity Cartesian angular velocity after assistance.
     * @param filtered_linear_velocity Final filtered linear velocity.
     * @param filtered_angular_velocity Final filtered angular velocity.
     * @param soft_goal Current soft goal.
     */
    void publishDebugData(const geometry_msgs::msg::Twist &latest_twist,
                          const Eigen::Vector3d &initial_filtered_linear_velocity,
                          const Eigen::Vector3d &initial_filtered_angular_velocity,
                          const Eigen::Vector3d &cartesian_linear_velocity,
                          const Eigen::Vector3d &cartesian_angular_velocity,
                          const Eigen::Vector3d &filtered_linear_velocity,
                          const Eigen::Vector3d &filtered_angular_velocity, const Goal &soft_goal);

    /**
     * @brief Implements the confidence integration mechanism from Algorithm 1
     * to estimate which goal the user is currently moving toward.
     *
     * Each real goal Gᵢ (i≥1) has a confidence cᵢ ∈ [0,1] updated as:
     *
     *   ẋcᵢ = α_conf ⋅ (‖v_E‖ / v_Jmax) ⋅ ((cosφ_W − cosθ_l) / (1 − cosθ_l))
     *
     * where:
     *   • cosφ_W = ( (W uᵢ)ᵀ (W v_E) ) / (‖W uᵢ‖ ‖W v_E‖)     weighted directional cosine
     *   • W = diag(1, 1, z_scale)                             anisotropic weighting (to reduce
     *   Z-sensitivity) • θ_l  = acceptance cone half-angle • α_conf = integration gain [s⁻¹]
     *
     * Integration occurs only if:
     *   • the user's motion speed ≥ v₁ (20 % threshold),
     *   • and the end-effector is outside all freeze spheres (r_freeze = min(r₁, r₂)).
     *
     * Otherwise, confidence updates are frozen.
     *
     * The agnostic goal G₀ keeps c₀ = max(0, 1 − Σ cᵢ), ensuring total confidence sums to 1.
     *
     * Notes:
     *   • θ-normalization ensures α_conf and θ_l are numerically decoupled.
     *   • Weighted cosine uses explicit z-scaling on both uᵢ and v_E (z_scale = 0.2).
     * @param dt Time step.
     * @param vE End-effector velocity.
     */
    void updateConfidences(double dt, const Eigen::Vector3d &vE);

    /**
     * @brief Weighted soft goal synthesis (Eq. 4):
     *   x_G = (∑ c_i x_i) / (∑ c_i)
     *   r_G = argmax_eig  of  A = ∑ c_i (q_i q_iᵀ)  (Markley quaternion average)
     * @return The computed soft goal.
     */
    Goal computeSoftGoal() const;

    /**
     * @brief Monitors whether the end-effector is within 6D reach of any known goal.
     * A goal is considered reached when both distance ≤ ε_t and orientation error ≤ ε_r
     * for at least dwell_ms milliseconds. When reached, a one-shot beep and goal index
     * are published; otherwise, a zero flag is sent each cycle. Prevents repeated triggers
     * using a dwell and latch mechanism.
     * @param now Current time.
     */
    void checkKnownGoalsAndBeep(const rclcpp::Time &now);

    /**
     * @brief Triggers reach feedback for a specific goal.
     * @param id Goal identifier.
     * @param distance Distance to the goal.
     */
    void trigger_reach_feedback(const std::string &id, double distance);

    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_; ///< Robot velocity interface component.

    // Stores the latest Twist command received.
    geometry_msgs::msg::Twist latest_twist_; ///< Latest received twist command.

    // Stores the previous initial filtered velocities
    Eigen::Vector3d previous_initial_filtered_linear_velocity_;  ///< Previous initial filtered linear velocity.
    Eigen::Vector3d previous_initial_filtered_angular_velocity_; ///< Previous initial filtered angular velocity.

    // Used for final velocity saturation
    Eigen::Vector3d previous_cartesian_linear_velocity_;  ///< Previous Cartesian linear velocity.
    Eigen::Vector3d previous_cartesian_angular_velocity_; ///< Previous Cartesian angular velocity.

    // Stores the previous post-filter velocities
    Eigen::Vector3d previous_filtered_linear_velocity_;  ///< Previous filtered linear velocity.
    Eigen::Vector3d previous_filtered_angular_velocity_; ///< Previous filtered angular velocity.

    // Robot state
    Eigen::Quaterniond current_orientation_; ///< Current end-effector orientation.
    Eigen::Vector3d current_position_;       ///< Current end-effector position.

    std::unordered_map<std::string, Goal> active_goals_; ///< Map of active goals by ID.
    std::unordered_map<std::string, Goal> param_goals_;  ///< Map of goals loaded from parameters.

    uint8_t mode_; ///< Current control mode (translation, rotation, etc.).

    // Parameters
    double alpha_conf_;    ///< α – confidence integration gain [s-1].
    double theta_l_;       ///< θ_l – half-cone aperture [rad].
    double r1_;            ///< r1 – inner dead-zone radius [m].
    double v_j_max_;       ///< max velocity fine manipulation [m/s].
    double gamma_;         ///< assistance gain on the parallel component.
    double r2_;            ///< outer radius for σ_d(d) where assistance saturates to 1.
    double theta1_;        ///< used in gate for assistance mode W.
    double theta2_;        ///< used in gate for assistance mode W.

    // Reach thresholds + dwell
    double eps_t_reach_;       ///< translational reach threshold [m].
    double eps_r_reach_rad_;   ///< rotational reach threshold [rad].
    int dwell_ms_;             ///< dwell time in milliseconds.

    // Dwell tracking
    std::string last_reached_goal_id_;  ///< ID of the last reached goal.
    rclcpp::Time reached_enter_time_;   ///< Time when reach condition was first met.
    bool beeped_for_this_visit_; ///< Flag to prevent multiple beeps per visit.

    // Flags to activate or desactivate functions
    bool enable_shared_control_assistance_; ///< Flag to enable/disable shared control assistance.
    bool enable_velocity_saturation_;       ///< Flag to enable/disable velocity saturation.
    bool enable_debug_publish_;             ///< Flag to enable/disable debug publishing.

    // Low-pass filters
    double initial_filter_cutoff_frequency_; ///< Cutoff frequency for initial low-pass filter [Hz].
    double final_filter_cutoff_frequency_;   ///< Cutoff frequency for final low-pass filter [Hz].
    bool lpf_initialized_;                   ///< Flag indicating if low-pass filters are initialized.

    // Cartesian velocity saturation
    double max_linear_delta_;  ///< Maximum allowed linear velocity change per second [m/s²].
    double max_angular_delta_; ///< Maximum allowed angular velocity change per second [rad/s²].

    // Input twist frame selection: "base" or "ee"
    std::string input_twist_frame_; ///< Frame for input twist commands ("base" or "ee").

    std::vector<std::string> command_names_; ///< Names of the command interfaces.

    //202603
    void goalsCallback(const visual_servoing::msg::DetectedGoalArray::SharedPtr msg);
    std::unordered_map<int, Goal> detected_goals_memory_;
    std::unordered_map<int, std::string> active_goal_key_by_tag_id_;
    //bool isSameDetectedGoal(const Goal &existing_goal,
                        //const geometry_msgs::msg::Pose &new_pose,
                        //double pos_tol = 0.02,
                        //double ang_tol_rad = 10.0 * M_PI / 180.0) const;
    bool isSameDetectedGoal(const Goal &existing_goal,
                        const geometry_msgs::msg::Pose &new_pose,
                        double pos_tol = 0,
                        double ang_tol_rad = 0) const;

    // Plotting publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr latest_twist_pub_; ///< Publisher for latest twist command.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr initial_filtered_linear_velocity_pub_;  ///< Publisher for initial filtered linear velocity.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr initial_filtered_angular_velocity_pub_; ///< Publisher for initial filtered angular velocity.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr assisted_linear_velocity_pub_;          ///< Publisher for assisted linear velocity.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr assisted_angular_velocity_pub_;         ///< Publisher for assisted angular velocity.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_linear_velocity_pub_;          ///< Publisher for final filtered linear velocity.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_angular_velocity_pub_;         ///< Publisher for final filtered angular velocity.

    // Goals and confidences publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr conf_pub_;      ///< Publisher for goal confidences.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr soft_goal_pub_;   ///< Publisher for soft goal pose.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr agnostic_goal_pub_; ///< Publisher for agnostic goal pose.
    // Publishers for python node
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr beep_pub_;                    ///< Publisher for beep feedback.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_reached_pub_;          ///< Publisher for goal reached signal (0/1 flag).
    // Mode publisher
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;                   ///< Publisher for current control mode.

    // Subscription for teleop Twist + Mode commands.
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr teleop_cmd_sub_; ///< Subscription for teleop commands.
    rclcpp::Subscription<extender_msgs::msg::SharedControlGoalArray>::SharedPtr goal_sub_; ///< Subscription for shared control goals.

    //202603
    rclcpp::Subscription<visual_servoing::msg::DetectedGoalArray>::SharedPtr goals_sub_;
    rclcpp::Publisher<visual_servoing::msg::SharedControlDebug>::SharedPtr shared_control_debug_pub_;

  };

} // namespace cartesian_velocity_controller
