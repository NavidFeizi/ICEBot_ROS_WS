#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <signal.h>
#include <random>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <boost/tokenizer.hpp>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For seeding the random number generator
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/startrecording.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "signal_generator.cpp"
#include "Butterworth.hpp"

using namespace std::chrono_literals;

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath);

void apply_sigmoid_smoothing(std::vector<double>& trajectory, double x1, double sample_time);

std::string package_name = "manager"; // Replace with any package in your workspace
std::string PACKAGE_SHARE_DIR = ament_index_cpp::get_package_share_directory(package_name);

enum class TargetType
{
  Multisine,
  Steps,
  Dataset,
  JointsFile,
  TaskFile
};

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode() : Node("manager"), count_(0)
  {
    ManagerNode::declare_parameters();
    ManagerNode::setup_ros_interfaces();
    // ManagerNode::create_dump_files();

    double f_cutoff = 20.0;
    m_filter = std::make_unique<ButterworthFilter<1>>(m_sample_time);
    m_filter->update_coeffs(f_cutoff);

    m_expt_time = 20.0 + 2.0;
    // m_expt_time = 240.0 + 2.0;
    // m_expt_time = 2.0 + 15.0;

    m_trajectory_type = TargetType::Dataset;

    switch (m_trajectory_type)
    {
    case TargetType::JointsFile: // from input file
    {
      std::string fileName("ICE_rotational_Actuation.csv");
      RCLCPP_INFO(this->get_logger(), "read joints target from input file mode - file name: %s", fileName);
      ManagerNode::load_input_files(m_joints_trajectory, fileName);
      m_traj_row = 1;
      ManagerNode::send_recod_request();
      break;
    }
    case TargetType::TaskFile: // from input file
    {
      std::string fileName("pseudo_random_trajectory.csv");
      // std::string fileName("trajectory_17.csv");
      RCLCPP_INFO(this->get_logger(), "read task target trajectory from input file mode - file name: %s", fileName);
      ManagerNode::load_task_input_files(m_task_trajectory, fileName);
      m_traj_row = 1;
      ManagerNode::send_recod_request();
      break;
    }
    case TargetType::Multisine: // from multisine generator
    {
      RCLCPP_INFO(this->get_logger(), "multiSine mode");
      ManagerNode::send_recod_request();
      break;
    }
    case TargetType::Steps: // from multisine generator
    {
      RCLCPP_INFO(this->get_logger(), "steps mode");
      ManagerNode::send_recod_request();
      break;
    }
    case TargetType::Dataset: // from random gnerator for dataset collection
    {
      int num_expt = 6;
      RCLCPP_INFO(this->get_logger(), "dataset collector mode");
      std::thread(&ManagerNode::dataset_expt, this, num_expt).detach();
      break;
    }
    default:
    {
      throw std::invalid_argument("Invalid TargetType specified.");
    }
    }

    rclcpp::Time now = this->get_clock()->now();
    t0_ = static_cast<double>(now.nanoseconds()) / 1E9;
    m_t = 0.0;

    RCLCPP_INFO(this->get_logger(), "manager node initialized");
  }

  ~ManagerNode()
  {
    if (m_is_experiment_running)
    {
      m_is_experiment_running = false;
      if (m_experiment_thread.joinable())
      {
        m_experiment_thread.join();
      }
    }
  }

private:
  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 4E-3);
    m_sample_time = get_parameter("sample_time").as_double();
    declare_parameter<bool>("task_space_mode", false);
    m_task_space_mode = get_parameter("task_space_mode").as_bool();
  }

  // Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    m_callback_group_pub = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_publisher_joint_space_target = this->create_publisher<interfaces::msg::Jointspace>("/joint_space/target", 10);     // to actuate the task space target generator model
    m_publisher_control_signal = this->create_publisher<interfaces::msg::Jointspace>("/joint_space/control_signal", 10); // to directly actuate the robot/simulation
    m_publisher_task_space_target = this->create_publisher<interfaces::msg::Taskspace>("/task_space/target", 10);        // to send task space target to the controller

    m_record_client = this->create_client<interfaces::srv::Startrecording>("start_recording"); // call to data recodring service of the recorder node
    while (!m_record_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Record service not available, waiting again...");
    }

    auto dt_us = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(dt_us, std::bind(&ManagerNode::target_joints_callback, this), m_callback_group_pub);
  }

  // Function to create and open dump files for recording
  void create_dump_files()
  {
    std::string package_name = "manager";
    std::string workspace_directory = ament_index_cpp::get_package_share_directory(package_name);
    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "manager-%Y-%m-%d_%H-%M-%S");
    std::string folder_address = workspace_directory + "/../../../../Output_Files/" + ss.str();
    std::filesystem::create_directories(folder_address);

    m_manager_dump_file.open(folder_address + "/" + "manager.csv");
    if (!m_manager_dump_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open joint_space.csv");
      return;
    }
    m_manager_dump_file << "t,q0,q1,q2,q3,q4,q5\n";

    RCLCPP_INFO(this->get_logger(), "Dump files opened - folder name: %s", ss.str().c_str());
  }

  // Function to load input CSV files
  void load_input_files(blaze::HybridMatrix<double, 60000UL, 5UL> &trajectory, const std::string &fileName)
  {
    std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
    ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
    std::filesystem::path filePath = ws_dir / "Input_Files" / fileName;
    readFromCSV(trajectory, filePath);
    RCLCPP_INFO(this->get_logger(), "Input file loaded");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  // Function to load input CSV files
  void load_task_input_files(blaze::HybridMatrix<double, 60000UL, 5UL> &trajectory, const std::string &fileName)
  {
    std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
    ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
    std::filesystem::path filePath = ws_dir / "Input_Files" / fileName;
    readFromCSV(trajectory, filePath);
    RCLCPP_INFO(this->get_logger(), "Input file loaded");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  // Send a record request to the recorder node over a service - the name of the file could be added to the service message
  void send_recod_request()
  {
    auto request = std::make_shared<interfaces::srv::Startrecording::Request>();
    request->duration = m_expt_time; // Set the desired duration

    using ServiceResponseFuture =
        rclcpp::Client<interfaces::srv::Startrecording>::SharedFuture;
    auto response_received_callback = std::bind(&ManagerNode::handle_recod_response, this, std::placeholders::_1);

    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  // Handle the response from the recorder node
  void handle_recod_response(rclcpp::Client<interfaces::srv::Startrecording>::SharedFuture future)
  {
    auto response = future.get();
    if (response->success)
    {
      // RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", response->message.c_str());
    }
  }

  // Run the experiment with random inputs signals multiple times for dataset generation
  void dataset_expt(int num_expt)
  {
    m_is_experiment_running = true;

    TrajectoryType traj_type = TrajectoryType::MultiSine; // or TrajectoryType::LinearDecrease   - CyclycNearPulse, MultiSine, CyclicNearStep
    // sample_time, num_waves , min_frequency[Hz], max_frequency[Hz] , min_amplitude, max_amplitude[m], total_samples
    MultiSineParams multiSineParams{m_sample_time, 7, 0.0, 1.5, 0.002, 0.006, m_expt_time};

    LinearDecreaseParams linearDecreaseParams{10.0, 50, 100, 50, 1000};
    // sample_time[s]; max_amplitude[m]; rise_duration[s]; unforced_time[s]; fall_vel[m/s]; cycle_period[s]; total_time[s];
    CyclicNearPulseParams cyclicNearPulseParams{m_sample_time, 0.005, 0.350, 1.0, 0.25, 2.0, m_expt_time};
    // sample_time[s]; max_amplitude[m]; switch_vel[m/s]; cycle_period[s]; total_time[s];
    CyclicNearStepParams cyclicNearStepParams{m_sample_time, 0.005, 0.1, 2.0, m_expt_time};

    std::unique_ptr<TrajectoryGenerator> m_signal_generator = createTrajectoryGenerator(traj_type,
                                                                                        multiSineParams,
                                                                                        linearDecreaseParams,
                                                                                        cyclicNearPulseParams,
                                                                                        cyclicNearStepParams);

    RCLCPP_INFO(this->get_logger(), "Dataset collecting experiment started");

    wait(2000);
    m_traj_row = 0;
    for (int i = 0; i < num_expt; ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Running experiment %d/%d", i + 1, num_expt);
      m_signal_generator->gen_trajectory();
      // m_signal_generator->flip_trajectory_sign();
      m_traj_AP = m_signal_generator->get_trajectory();
      apply_sigmoid_smoothing(m_traj_AP, 1.0, m_sample_time);

      m_signal_generator->gen_trajectory();
      // m_signal_generator->flip_trajectory_sign();
      m_traj_RL = m_signal_generator->get_trajectory();
      apply_sigmoid_smoothing(m_traj_RL, 1.0, m_sample_time);

      ManagerNode::send_recod_request();
      m_traj_row = 0;
      while (m_traj_row < m_traj_AP.size())
      {
        wait(4);
      }
      wait(1000);
    }

    RCLCPP_INFO(this->get_logger(), "Dataset collecting experiment finished");
  }

  // Timer callback function to publish the target joints config
  void target_joints_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    m_t = m_t + m_sample_time; // current local time

    switch (m_trajectory_type)
    {
    case TargetType::JointsFile:
    {
      if (m_traj_row < m_joints_trajectory.rows()) // Ensure we are not exceeding the number of rows
      {
        m_X = {m_joints_trajectory(m_traj_row, 1),  // Column 1 (index 0)
               m_joints_trajectory(m_traj_row, 3),  // Column 2 (index 1)
               m_joints_trajectory(m_traj_row, 2),  // Column 3 (index 2)
               m_joints_trajectory(m_traj_row, 4)}; // Column 4 (index 3)
        // x_temp = m_filter->add_data_point(x_temp);                                   // Assuming this function is correctly handling a StaticVector of size 1
        // m_X = -1 * x_temp; // Store the filtered value
        ++m_traj_row; // Increment row index for next call
      }
      break;
    }
    case TargetType::TaskFile:
    {
      auto msg = interfaces::msg::Taskspace();
      blaze::StaticVector<float, 5> row_data = {0.0, 0.0, 0.0, 0.0, 0.0};
      if (m_traj_row < m_task_trajectory.rows()) // Ensure we are not exceeding the number of rows
      {
        for (size_t col = 0; col < m_task_trajectory.columns(); ++col)
        {
          row_data[col] = m_task_trajectory(m_traj_row, col); // Scale values by 1000
        }
        ++m_traj_row; // Increment row index for next call
      }
      msg.p[0] = row_data[1] * 1e0; // position_x
      msg.p[1] = 0.0;               // position_y
      msg.p[2] = row_data[3] * 1e0; // position_z
      msg.h[0] = 0.0;               // orientation quat
      msg.h[1] = 0.0;               // orientation quat
      msg.h[2] = 0.0;               // orientation quat
      msg.h[3] = 0.0;               // orientation quat
      msg.q[0] = 0.0;               // velocity_x
      msg.q[1] = 0.0;               // velocity_x
      msg.q[2] = 0.0;               // velocity_z
      msg.w[0] = 0.0;               // angular velocity
      msg.w[1] = 0.0;               // angular velocity
      msg.w[2] = 0.0;               // angular velocity
      m_publisher_task_space_target->publish(msg);

      break;
    }
    case TargetType::Multisine:
    {
      ManagerNode::multi_sine(m_t, m_x);
      // ManagerNode::ramp(m_t, m_x);
      break;
    }
    case TargetType::Steps:
    {
      // ManagerNode::multi_steps_signal(m_t, m_x);

      // for direct target generation --- must be corrected later
      auto msg = interfaces::msg::Taskspace();

      blaze::StaticVector<float, 4> x = {-1.66, 0.0, 54.32, 0.0};
      blaze::StaticVector<double, 3> m_switch_times = {5.0, 10.0, 15.0};
      double cycle_time = fmod(m_t, m_switch_times[2]);

      if (cycle_time >= 0 && cycle_time < m_switch_times[0])
      {
        x = {-1.66, 0.0, 54.32, 0.0};
      }
      else if (cycle_time >= m_switch_times[0] && cycle_time < m_switch_times[1])
      {
        x = {28.05, 0.0, 42.85, 0.0};
      }
      else
      {
        x = {-30.05, 0.0, 41.02, 0.0};
      }

      // publish tip states
      msg.p[0] = x[1] * 1e-3; // position_x
      msg.p[1] = 0.0;         // position_y
      msg.p[2] = x[3] * 1e-3; // position_z
      msg.h[0] = 0.0;         // orientation quat
      msg.h[1] = 0.0;         // orientation quat
      msg.h[2] = 0.0;         // orientation quat
      msg.h[3] = 0.0;         // orientation quat
      msg.q[0] = 0.0;         // velocity_x
      msg.q[1] = 0.0;         // velocity_x
      msg.q[2] = 0.0;         // velocity_z
      msg.w[0] = 0.0;         // angular velocity
      msg.w[1] = 0.0;         // angular velocity
      msg.w[2] = 0.0;         // angular velocity
      m_publisher_task_space_target->publish(msg);

      break;
    }
    case TargetType::Dataset:
    {
      if (m_traj_row < m_traj_AP.size())
      {
        m_X[0] = {m_traj_AP[m_traj_row]};
        m_X[1] = {m_traj_RL[m_traj_row]};
        // x_temp = m_filter->add_data_point(x_temp);
        // m_x = x_temp[0];
        // m_X[0] = m_x;
        // m_X[1] = m_x;
        ++m_traj_row;
      }
      break;
    }
    default:
      std::cout << "Invalid choice. Please enter 1, 2, or 3.\n";
      break;
    }

    // constexpr double period = 5.0;     // Period of the pulse signal
    // constexpr double amplitude = 5.0; // [mm] Amplitude of the pulse signal
    // constexpr double velocity = 60.0;  // [mm/s] Velocity of the ramp rise and fall
    // ManagerNode::pulse_signal(period, amplitude, velocity, m_t, x);

    // // // calcualte velocity signal
    // // double x_dot = (m_x - m_x_prev) / m_sample_time;
    // // m_x_prev = m_x;
    // m_X[0] = m_x;
    // m_X[1] = 0.0;
    // m_X[2] = 0.0;
    // m_X[3] = 0.0;

    // // To slack the antagonist tendon
    m_msg.position[0UL] = 0.0;                                               // insertion
    m_msg.position[1UL] = 0.0;                                               // rotation
    m_msg.position[2UL] = (m_X[0] <= 0) ? (m_X[0]) : (0.3 * m_X[0]);         // anterior (positive is pull)
    m_msg.position[3UL] = (m_X[0] >= 0) ? (-1.0 * m_X[0]) : (-0.3 * m_X[0]); // posterior (positive is pull)
    m_msg.position[4UL] = (m_X[1] <= 0) ? (m_X[1]) : (0.3 * m_X[1]);         // right
    m_msg.position[5UL] = (m_X[1] >= 0) ? (-1.0 * m_X[1]) : (-0.3 * m_X[1]); // left

    // m_msg.position[0UL] = 0.0; // insertion
    // m_msg.position[1UL] = 0.0;        // rotation
    // m_msg.position[2UL] = -1*m_X[0];           // anterior (positive is pull)
    // m_msg.position[3UL] = -1*m_X[1];           // posterior (positive is pull)
    // m_msg.position[4UL] = -1*m_X[2];           // right
    // m_msg.position[5UL] = -1*m_X[3];           // left

    m_msg.velocity[0UL] = 0.0 * 1e-3; // insertion
    m_msg.velocity[1UL] = 0.0;        // rotation
    m_msg.velocity[2UL] = 0.0 * 1e-3; // anterior (positive is pull)
    m_msg.velocity[3UL] = 0.0 * 1e-3; // posterior (positive is pull)
    m_msg.velocity[4UL] = 0.0 * 1e-3; // right
    m_msg.velocity[5UL] = 0.0 * 1e-3; // left

    // // if (m_t>= 0 && m_t<22.0)
    // // {
    // m_manager_dump_file << std::fixed << std::setprecision(3)
    //                     << m_t << ","
    //                     << std::fixed << std::setprecision(7)
    //                     << m_msg.position[0] << ',' << m_msg.position[1] << ',' << m_msg.position[2] << ',' << m_msg.position[3] << ',' << m_msg.position[4] << ',' << m_msg.position[5] << '\n';
    // // }

    if (m_task_space_mode)
    {
      m_publisher_joint_space_target->publish(m_msg);
    }
    else
    {
      m_publisher_control_signal->publish(m_msg);
    }

    // RCLCPP_INFO(this->get_logger(), "new target: ")
    // std::cout << std::fixed << std::setprecision(4);
    // std::cout << m_msg.position[0UL] << std::endl;
  }

  // Function that generates a custom multi-sine/sinusoidal signal
  void multi_sine(const double t, double &x)
  {
    blaze::StaticVector<double, 6> period_list = {1, 2, 4, 10, 20, 40};
    blaze::StaticVector<double, 6> freq_list = {0.6, 1.0677, 1.7333, 2.2677, 2.4677, 3.3333};
    freq_list = freq_list * 0.5;

    // double amplitude = 3.4;
    // double frequency = 0.4;

    // for RAL
    double frequency = 0.2;
    double amplitude = -0.005;

    if (t >= 2 && t < m_expt_time - 2.0)
    {
      // for RAL
      //  x = amplitude * (0.0 + 1.0 * (std::sin(2 * M_PI * 1.0 * (t - 1))));

      // // for RAL
      x = amplitude * (0.0 + 1.0 * (std::sin(2 * M_PI * frequency * 1 / 1 * (t - 2))));
      // x += amplitude * (0.0 + 0.3 * (std::sin(2 * M_PI * frequency * 1 / 3 * (t - 1))));
      // x += amplitude * (0.0 + 0.2 * (std::sin(2 * M_PI * frequency * 1 / 1 * (t - 1))));

      // //for RAL
      // int i = static_cast<int>(floor((t - 1) / 40));
      // double frequency = 1.0 / period_list[i];
      // RCLCPP_INFO(this->get_logger(), "period is %0.2f", period_list[i]);
      // x = amplitude * 1.0 * (0.0 + (std::sin(2 * M_PI * frequency * (t - 1) )));

      // // general muilti sine
      // x = amplitude * 0.3 * (0.0 + (std::sin(2 * M_PI * freq_list[0] * (t - 1) - M_PI_2)));   // Sinusoidal value at time t
      // x += amplitude * 0.15 * (0.0 + (std::sin(2 * M_PI * freq_list[1] * (t - 1) - M_PI_2))); // Additional sinusoidal value at time t
      // x += amplitude * 0.15 * (0.0 + (std::sin(2 * M_PI * freq_list[2] * (t - 1) - M_PI_2))); // Additional sinusoidal value at time t
      // x += amplitude * 0.2 * (0.0 + (std::sin(2 * M_PI * freq_list[3] * (t - 1) - M_PI_2)));  // Additional sinusoidal value at time t
      // x += amplitude * 0.1 * (0.0 + (std::sin(2 * M_PI * freq_list[4] * (t - 1) - M_PI_2)));  // Additional sinusoidal value at time t
      // x += amplitude * 0.1 * (0.0 + (std::sin(2 * M_PI * freq_list[5] * (t - 1) - M_PI_2)));  // Additional sinusoidal value at time t
    }
    else
    {
      x = 0.0;
    }
  }

  //
  void ramp(const double t, double &x)
  {
    double amplitude = 6.0;
    double duration = 10;
    double slope = amplitude / duration * 4;

    double cycle_time = fmod(t, duration);

    if (cycle_time >= 0 && cycle_time < duration / 4)
    {
      x = cycle_time * slope;
    }
    else if (cycle_time >= duration / 4 && cycle_time < duration * 3 / 4)
    {
      x = amplitude + (cycle_time - duration / 4) * -1 * slope;
    }
    else
    {
      x = -1 * amplitude + (cycle_time - duration * 3 / 4) * slope;
    }
  }

  //
  void multi_steps_signal(const double t, double &x)
  {
    blaze::StaticVector<double, 3> m_switch_times = {5.0, 10.0, 15.0};
    double cycle_time = fmod(t, m_switch_times[2]);
    double amplitude = 5;

    if (cycle_time >= 0 && cycle_time < m_switch_times[0])
    {
      x = 0.0; // 0 for 0 < time < switch_time1
    }
    else if (cycle_time >= m_switch_times[0] && cycle_time < m_switch_times[1])
    {
      x = amplitude; // Amplitude for switch_time1 < time < switch_time2
    }
    else
    {
      x = -1 * amplitude; // -Amplitude for switch_time2 < time < switch_time3
    }
  }

  // Generates a pulse-like signal with smooth transitions
  void pulse_signal(const double period, const double amplitude, const double velocity, const double t, double &x)
  {

    double half_period = period / 2.0;
    double ramp_time = amplitude / velocity; // Time taken for the ramp to rise or fall

    // Calculate the time within the current period
    double mod_time = fmod(t, period);

    // Generate the signal
    if (mod_time < ramp_time)
    {
      x = (velocity * mod_time); // Rising ramp
    }
    else if (mod_time >= ramp_time && mod_time < half_period)
    {
      x = amplitude; // High state
    }
    else if (mod_time >= half_period && mod_time < (half_period + ramp_time))
    {
      x = amplitude - velocity * (mod_time - half_period); // Falling ramp
    }
    else if (mod_time >= (half_period + ramp_time) && mod_time < period)
    {
      x = 0; // Low state
    }
    else
    {
      // Rising ramp again (since we are in the next period's ramp-up phase)
      x = (velocity * (mod_time - (period - ramp_time)));
    }
  }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.0 / milliseconds);
    rate.sleep();
  }

  // member variables
  size_t count_;
  double m_sample_time = 10e-3; //[s]
  double t0_, m_t, t0 = 0;

  double m_expt_time;
  double m_x, m_x_prev = 0.0;
  blaze::StaticVector<double, 4> m_X, m_X_prev;

  bool m_task_space_mode;
  TargetType m_trajectory_type;

  std::vector<double> m_traj_AP, m_traj_RL;
  long unsigned int m_traj_row = 1;
  blaze::HybridMatrix<double, 60000UL, 5UL> m_joints_trajectory; // time, T1,T2,T3,T4
  blaze::HybridMatrix<double, 60000UL, 5UL> m_task_trajectory;
  std::unique_ptr<ButterworthFilter<1>> m_filter;

  interfaces::msg::Jointspace m_msg;

  std::ofstream m_manager_dump_file;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_joint_space_target, m_publisher_control_signal;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_task_space_target; // publisher object
  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub;
  rclcpp::Client<interfaces::srv::Startrecording>::SharedPtr m_record_client;

  std::atomic<bool> m_is_experiment_running;
  std::thread m_experiment_thread;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// Sigmoid function
double sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

// Function to apply sigmoid smoothing
void apply_sigmoid_smoothing(std::vector<double>& trajectory, double x1, double sample_time) {
    int size = trajectory.size();
    if (size < 2) return;

    // Number of points to smooth based on x1 and sample time
    int smooth_points = static_cast<int>(x1 / sample_time);
    if (smooth_points <= 0 || smooth_points * 2 >= size) {
        std::cerr << "Smoothing region is too large or too small for the trajectory size." << std::endl;
        return;
    }

    double scaling_factor = 10.0;  // Controls sigmoid steepness

    // Apply sigmoid at the beginning
    for (int i = 0; i < smooth_points; ++i) {
        double normalized_index = static_cast<double>(i) / smooth_points;
        double sigmoid_value = sigmoid(scaling_factor * (normalized_index - 0.5));
        trajectory[i] *= sigmoid_value;
    }

    // Apply flipped sigmoid at the end
    for (int i = size - smooth_points; i < size; ++i) {
        double normalized_index = static_cast<double>(i - (size - smooth_points)) / smooth_points;
        double sigmoid_value = sigmoid(scaling_factor * (0.5 - normalized_index));
        trajectory[i] *= sigmoid_value;
    }
}


// function that reads data from CSV files
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath)
{
  std::ifstream CSV_file;
  CSV_file.open(filePath, std::ifstream::in);
  if (!CSV_file.is_open())
  {
    throw std::runtime_error("Error opening the CSV file: " + filePath.string());
  }

  typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

  std::string line;
  // Skip the first line (header)
  if (!std::getline(CSV_file, line))
  {
    std::cerr << "Error reading the header line\n";
    return Mat = -1.00;
  }

  size_t row = 0UL, col = 0UL;
  double value;

  while (std::getline(CSV_file, line))
  {
    Tokenizer tokenizer(line);
    col = 0UL;
    for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
    {
      value = std::stod(*it);
      Mat(row, col) = value;
      ++col;
    }
    ++row;
  }

  CSV_file.close();
  Mat.resize(row, col, true);

  return Mat;
}