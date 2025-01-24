#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "CatheterRobot.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void print_robot_status(double t, blaze::StaticVector<double, 6> position, blaze::StaticVector<double, 6> velocity, blaze::StaticVector<double, 6> current);

class RobotNode : public rclcpp::Node
{
public:
  RobotNode() : Node("catheter_robot"), m_count(0)
  {
    declare_parameters();
    setup_and_initialize_robot();
    setup_ros_interfaces();
    setup_parameter_callback();
  }

private:
  // Declare ROS parameters
  void declare_parameters()
  {
    // Declare parameters as vectors with 6 default values
    declare_parameter<std::vector<double>>("Kp", std::vector<double>{20.0, 20.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("Ki", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Get parameter values
    m_kp = get_parameter("Kp").as_double_array();
    m_ki = get_parameter("Ki").as_double_array();

    for (size_t i = 0; i < m_ki.size(); ++i)
    {
      m_kp_vec[i] = m_kp[i];
    }
    for (size_t i = 0; i < m_ki.size(); ++i)
    {
      m_ki_vec[i] = m_ki[i];
    }

    m_home_pre_tension = 1.0; //[mm]
  }

  // Setup and initialize the robot
  void setup_and_initialize_robot()
  {
    // int8_t operation_mode = static_cast<int8_t>(0xFE); // 0x01, 0x03, 0xFE
    // bool position_limit = false;
    // blaze::StaticVector<double, 6UL> max_vel = {8.0E-3, 0.2, 10.0E-3, 10.0E-3, 10.0E-3, 10.0E-3};
    // blaze::StaticVector<double, 6UL> max_acc = {15.0E-3, 0.6, 160.0E-3, 160.0E-3, 160.0E-3, 160.0E-3};
    // m_robot = std::make_unique<Robot>(
    //     static_cast<int>(m_control_sample_time * 1e6),
    //     operation_mode,
    //     max_acc, max_vel,
    //     position_limit);

    blaze::StaticVector<double, 6UL> max_acc = {5.0E-3, 10.0, 50.0E-3, 50.0E-3, 50.0E-3, 50.0E-3};  // SI
    blaze::StaticVector<double, 6UL> max_vel = {20.0E-3, 20.0, 50.0E-3, 50.0E-3, 50.0E-3, 50.0E-3}; // SI
    blaze::StaticVector<OpMode, 6UL> operation_mode = {OpMode::VelocityProfile, OpMode::VelocityProfile, OpMode::PositionProfile, OpMode::PositionProfile, OpMode::PositionProfile, OpMode::PositionProfile};
    bool position_limit = false;

    m_robot = std::make_unique<Robot>(static_cast<int>(m_control_sample_time * 1e3), operation_mode, max_acc, max_vel, position_limit);

    m_robot->Start_Thread();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (m_robot->Get_Controller_Switch_Status())
    {
      // m_robot->Enable_Operation(true);
    }
    m_robot->Set_Target_Position({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Setup ROS interfaces including publishers, subscribers, services, and timers
  void setup_ros_interfaces()
  {
    // Create callback groups to ensure mutually exclusive callbacks
    m_callback_group_pub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_homing = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_watchdog_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_watchdog_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target joint configurations
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub1;
    m_subscription_target = create_subscription<interfaces::msg::Jointspace>(
        "/joint_space/control_signal", 10, std::bind(&RobotNode::target_callback, this, _1), subs_options_1);

    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub2;
    m_subscription_emt = create_subscription<interfaces::msg::Taskspace>(
        "/task_space/feedback", 10, std::bind(&RobotNode::current_catheter_tip_callback, this, _1), subs_options_2);

    // Publisher to broadcast the current joint configurations
    m_publisher_robot = create_publisher<interfaces::msg::Jointspace>("/joint_space/feedback", 10);

    // Low-level control loop timer
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1e6));
    m_control_loop_timer = create_wall_timer(control_sample_time, std::bind(&RobotNode::joint_space_control_callback, this), m_callback_group_pub1);

    // Timer to read joint configurations periodically
    m_read_robot_timer = create_wall_timer(1ms, std::bind(&RobotNode::joints_config_callback, this), m_callback_group_read);

    // Initialize homing service
    m_homing_service = this->create_service<std_srvs::srv::Trigger>(
        "homing", std::bind(&RobotNode::homing_callback, this, _1, std::placeholders::_2));

    // Initialize the watchdog timer
    m_watchdog_timer_emt = this->create_wall_timer(100ms, std::bind(&RobotNode::check_emtracker_alive, this), m_callback_group_watchdog_1);
    m_watchdog_timer_target = this->create_wall_timer(2000ms, std::bind(&RobotNode::check_target_publisher_alive, this), m_callback_group_watchdog_2);
  }

  // Setup parameter callback function to handle dynamic parameter updates
  void setup_parameter_callback()
  {
    auto param_callback = [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "Kp")
        {
          m_kp = parameter.as_double_array();
          std::ostringstream oss;     // Log the vector
          oss << "[";
          for (size_t i = 0; i < m_ki.size(); ++i)
          {
            m_kp_vec[i] = m_kp[i];
            m_ki_vec[i] = m_ki[i];
            oss << m_ki[i];
            if (i != m_ki.size() - 1) oss << ", "; // Add a comma except for the last element
          }   
          oss << "]";
          RCLCPP_INFO(get_logger(), "Parameter 'Kp' set to: %s", oss.str().c_str());
        }
        else if (parameter.get_name() == "Ki")
        {
          m_ki = parameter.as_double_array();
          std::ostringstream oss;     // Log the vector
          oss << "[";
          for (size_t i = 0; i < m_ki.size(); ++i)
          {
            m_ki_vec[i] = m_ki[i];
            oss << m_ki[i];
            if (i != m_ki.size() - 1) oss << ", "; // Add a comma except for the last element
          }   
          oss << "]";
          RCLCPP_INFO(get_logger(), "Parameter 'Ki' set to: %s", oss.str().c_str());
        }
        else
        {
          result.successful = false;
          result.reason = "Unknown parameter";
          RCLCPP_WARN(get_logger(), "Unknown parameter '%s'", parameter.get_name().c_str());
        }
      }

      return result;
    };

    m_param_callback_handle = add_on_set_parameters_callback(param_callback);
  }

  // Timer callback function to read the current joint configurations and publish them
  void joints_config_callback()
  {
    auto msg = interfaces::msg::Jointspace();
    // rclcpp::Time now = get_clock()->now();
    // double time = static_cast<double>(now.nanoseconds()) / 1E9;

    m_robot->Get_PosVelCur(&m_x_abs, &m_x, &m_x_dot, &m_c);

    msg.position_abs[0] = m_x_abs[0];
    msg.position_abs[1] = m_x_abs[1];
    msg.position_abs[2] = m_x_abs[2];
    msg.position_abs[3] = m_x_abs[3];
    msg.position_abs[4] = m_x_abs[4];
    msg.position_abs[5] = m_x_abs[5];

    msg.position[0] = m_x[0];
    msg.position[1] = m_x[1];
    msg.position[2] = m_x[2];
    msg.position[3] = m_x[3];
    msg.position[4] = m_x[4];
    msg.position[5] = m_x[5];

    msg.velocity[0] = m_x_dot[0];
    msg.velocity[1] = m_x_dot[1];
    msg.velocity[2] = m_x_dot[2];
    msg.velocity[3] = m_x_dot[3];
    msg.velocity[4] = m_x_dot[4];
    msg.velocity[5] = m_x_dot[5];

    m_publisher_robot->publish(msg);
  }

  // Timer callback function for joint space control loop
  void joint_space_control_callback()
  {
    blaze::StaticVector<double, 6UL> q_dot_command, x_des, x_dot_des;

    if (!m_flag_homing)
    {
      x_des = m_x_des;
      x_dot_des = m_x_dot_des;
      joint_space_control_step(x_des, x_dot_des, q_dot_command);

      m_robot->Set_Target_Position(x_des);     // tendons
      m_robot->Set_Target_Velocity(q_dot_command); // rotation and insertion
      // std::cout << "q_dot_command: " << q_dot_command <<std::endl
    }
  }

  // Perform a control step in joint space
  void joint_space_control_step(const blaze::StaticVector<double, 6UL> x_des, const blaze::StaticVector<double, 6UL> x_dot_des, blaze::StaticVector<double, 6UL> &q_dot_command)
  {
    m_x_error = x_des - m_x;
    m_x_error_int = m_x_error_int + m_x_error * m_control_sample_time;
    q_dot_command = m_x_error * m_kp_vec + m_x_error_int * m_ki_vec + x_dot_des;
  }

  // Set the target position in the robot - Depreciated
  void position_command_loop()
  {
    m_robot->Set_Target_Position(m_x_des);
  }

  // Perform system identification - Depreciated
  void system_identification_loop()
  {
    m_robot->Set_Target_Velocity(m_x_dot_des);
    std::cout << "\r" << std::dec << std::fixed << std::setprecision(3) << "q2_dot:" << m_x_dot_des[2] << std::endl;
  }

  // Subscription callback function to updates the target joint positions and velocities
  void target_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    m_targpublisher_alive_tmep = true;
    m_x_des = blaze::StaticVector<double, 6UL>(0.0);

    m_x_des[0] = msg->position[0];
    m_x_des[1] = msg->position[1];
    m_x_des[2] = -1 * msg->position[2];
    m_x_des[3] = -1 * msg->position[3];
    m_x_des[4] = -1 * msg->position[4];
    m_x_des[5] = -1 * msg->position[5];

    m_x_dot_des[0] = msg->velocity[0];
    m_x_dot_des[1] = msg->velocity[1];
    m_x_dot_des[2] = -1 * msg->velocity[2];
    m_x_dot_des[3] = -1 * msg->velocity[3];
    m_x_dot_des[4] = -1 * msg->velocity[4];
    m_x_dot_des[5] = -1 * msg->velocity[5];

    // RCLCPP_INFO(this->get_logger(), "New Target");
  }

  // Subscription callback function updates the current catheter tip status using EMTracker topic
  void current_catheter_tip_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_catheter = {msg->p[0] * 1e3,
                    msg->p[1] * 1e3,
                    msg->p[2] * 1e3};
    m_emtracker_alive_tmep = true;
  }

  // Service callback to perform the homing procedure
  void homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // Explicitly mark it as unused

    if (m_emtracker_alive)
    {
      m_flag_homing = true;
      blaze::StaticVector<double, 6UL> motion_inc, motion_vel, current_pos_abs, current_pos;
      RCLCPP_INFO(get_logger(), "Homing...");
      motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      current_pos_abs = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
      m_robot->Get_Position_Abs(&current_pos_abs);

      m_robot->Set_Target_Position_Abs(current_pos_abs);
      wait(100);

      // A-P plane
      RCLCPP_INFO(get_logger(), "->  Pulling anterior tendon");
      motion_vel = {0.0, 0.0, 0.0004, 00.0, 0.0, 0.0}; 
      motion_inc = motion_vel * 0.01;
      while (m_x_catheter[0] < m_home_pre_tension)
      {
        current_pos_abs += motion_inc;
        m_robot->Set_Target_Position_Abs(current_pos_abs);
        wait(10);
      }
      RCLCPP_INFO(get_logger(), "->  Pulling posterior tendon");
      motion_vel = {0.0, 0.0, 0.0, 0.0002, 0.0, 0.0};
      motion_inc = motion_vel * 0.01;
      while (m_x_catheter[0] > 0.3)
      {
        current_pos_abs += motion_inc;
        m_robot->Set_Target_Position_Abs(current_pos_abs);
        wait(10);
      }

      // R-L plane
      if (m_x_catheter[1] < 0.0)
      {
        RCLCPP_INFO(get_logger(), "->  Pulling right tendon");
        motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0004, 0.0}; 
        motion_inc = motion_vel * 0.01;
        while (m_x_catheter[1] < m_home_pre_tension)
        {
          current_pos_abs += motion_inc;
          m_robot->Set_Target_Position_Abs(current_pos_abs);
          wait(10);
        }
        RCLCPP_INFO(get_logger(), "->  Pulling left tendon");
        motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0002};
        motion_inc = motion_vel * 0.01;
        while (m_x_catheter[1] > 0.3)
        {
          current_pos_abs += motion_inc;
          m_robot->Set_Target_Position_Abs(current_pos_abs);
          wait(10);
        }
      }
      else if (m_x_catheter[1] > 0.0)
      {
        RCLCPP_INFO(get_logger(), "->  Pulling left tendon");
        motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0004};
        motion_inc = motion_vel * 0.01;
        while (m_x_catheter[1] > -1 * m_home_pre_tension)
        {
          current_pos_abs += motion_inc;
          m_robot->Set_Target_Position_Abs(current_pos_abs);
          wait(10);
        }
        RCLCPP_INFO(get_logger(), "->  Pulling right tendon");
        motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0002, 0.0}; 
        motion_inc = motion_vel * 0.01;
        while (m_x_catheter[1] < -0.3)
        {
          current_pos_abs += motion_inc;
          m_robot->Set_Target_Position_Abs(current_pos_abs);
          wait(10);
        }
      }

      // Translation
      if (m_x_catheter[2] < 59.0)
      {
        RCLCPP_INFO(get_logger(), "->  Inserting");
        motion_vel = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        while (m_x_catheter[2] < 59.0)
        {
          m_robot->Set_Target_Velocity(motion_vel);
          // std::cout << current_pos_abs << std::endl;
          wait(10);
        }
      }
      else if (m_x_catheter[2] > 59.5)
      {
        RCLCPP_INFO(get_logger(), "->  Retracting");
        motion_vel = {-0.001, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        while (m_x_catheter[2] > 59.5)
        {
          m_robot->Set_Target_Velocity(motion_vel);
          wait(10);
        }
      }

      motion_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
      m_robot->Set_Target_Velocity(motion_vel);

      // wait(100);
      // q_dot_command = {0.0, 0.0, -0.001, -0.001, 0.0, 0.0};
      // m_robot->Set_Target_Velocity(q_dot_command);
      // wait(4000);
      // q_dot_command = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // m_robot->Set_Target_Velocity(q_dot_command);
      // wait(100);
      current_pos = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
      m_robot->Set_Soft_Home(current_pos);
      m_robot->Set_Target_Position(current_pos);

      response->success = true;
      response->message = "Homing completed successfully";
      RCLCPP_INFO(get_logger(), "Homing finished");
      m_flag_homing = false;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Homing not started - emtracker_node is dead - catheter tip position info is required for homing");
      response->success = false;
      response->message = "Homing not started - emtracker_node is dead";
    }
  }

  // Timer callback function to check if the EMTracker is still alive
  void check_emtracker_alive()
  {
    // Set the flag to false; it will be set to true if current_catheter_tip_callback is called
    if (m_emtracker_alive_tmep)
    {
      if (!m_emtracker_alive)
      {
        m_emtracker_alive = true;
        RCLCPP_WARN(get_logger(), "emtracker_node is alive");
      }
    }
    else
    {
      if (m_emtracker_alive)
      {
        m_emtracker_alive = false;
        RCLCPP_WARN(get_logger(), "emtracker_node is dead");
      }
    }
    m_emtracker_alive_tmep = false;
  }

  // Timer callback function to check if the EMTracker is still alive
  void check_target_publisher_alive()
  {
    // Set the flag to false; it will be set to true if current_catheter_tip_callback is called
    if (m_targpublisher_alive_tmep)
    {
      if (!m_targpublisher_alive)
      {
        m_targpublisher_alive = true;
        RCLCPP_WARN(get_logger(), "publisher_node is alive");
      }
    }
    else
    {
      if (m_targpublisher_alive)
      {
        m_targpublisher_alive = false;
        m_x_des = blaze::StaticVector<double, 6UL>(0.0);
        m_x_dot_des = blaze::StaticVector<double, 6UL>(0.0);
        RCLCPP_WARN(get_logger(), "publisher_node is dead");
      }
    }
    m_targpublisher_alive_tmep = false;
  }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.0 / milliseconds);
    rate.sleep();
  }

  size_t m_count;
  const double m_control_sample_time = 0.010; //[s]
  std::vector<double> m_kp;
  std::vector<double> m_ki;
  blaze::StaticVector<double, 6UL> m_kp_vec;
  blaze::StaticVector<double, 6UL> m_ki_vec;
  bool m_flag_homing = false;
  bool m_emtracker_alive, m_emtracker_alive_tmep, m_targpublisher_alive, m_targpublisher_alive_tmep = false;
  double m_home_pre_tension;
  std::unique_ptr<Robot> m_robot;

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_emt;
  rclcpp::TimerBase::SharedPtr m_watchdog_timer_target;
  rclcpp::TimerBase::SharedPtr m_read_robot_timer;
  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  rclcpp::TimerBase::SharedPtr m_position_control_timer;
  rclcpp::TimerBase::SharedPtr m_system_identification_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_robot;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_emt;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_homing_service;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub2;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_homing;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_2;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  blaze::StaticVector<double, 6UL> m_x_abs = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_dot = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_c = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_des = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_error = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_error_int = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_error_dot = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_dot_forward = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_dot_des = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 6UL> m_x_des_home = blaze::StaticVector<double, 6UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_catheter = blaze::StaticVector<double, 3UL>(0.0);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

void print_robot_status(double t, blaze::StaticVector<double, 6> position, blaze::StaticVector<double, 6> velocity, blaze::StaticVector<double, 6> current)
{
  auto print_with_space_if_positive = [](double value)
  {
    if (value >= 0)
    {
      std::cout << " " << value;
    }
    else
    {
      std::cout << value;
    }
  };

  std::cout << "\r" << std::dec << std::fixed << std::setprecision(3) << "Robot => Time: " << t << "[s]" << std::endl;
  std::cout << "Units: [mm], [rad], [mm/s], [rad/s], [A]" << std::endl;
  for (size_t i = 0; i < 6; i++)
  {
    std::cout << "Node " << i + 1 << " =>  ";
    std::cout << "Pos: ";
    std::cout << std::fixed << std::setprecision(5);
    print_with_space_if_positive(position[i] * 1e3);
    std::cout << "     Vel: ";
    print_with_space_if_positive(velocity[i] * 1e3);
    std::cout << "     Current: ";
    print_with_space_if_positive(current[i]);
    std::cout << " \n";
  }
  std::cout << std::endl;
}