#include "CatheterRobot.hpp"

using namespace std::chrono_literals;
using namespace lely;

std::vector<double> Position_Target_Generator(double t);

Robot::Robot(int sample_time,
             blaze::StaticVector<OpMode, 6UL> operation_mode,
             blaze::StaticVector<double, 6UL> max_acc,
             blaze::StaticVector<double, 6UL> max_vel,
             bool position_limit)
{
  this->m_enable_joint = {1, 1, 1, 1, 1, 1}; // set the master.yml accordingly
  this->encoder_res = {1000 * 1000.0,
                       10 * 180 * M_1_PI,
                       3000, 3000, 3000, 3000};          // pulse/rev
  this->gear_ratio = {1, 1, 18E-3/3, 18E-3/3, 18E-3/3, 18E-3/3}; // rad->rev or m->rev - (gearbox and belt is integrated in the motion controller parameters)
  this->max_acc = max_acc;                               // deg->rev or mm->rev
  this->max_vel = max_vel;                               // deg->rev or mm->rev
  this->velocity_factor = {10.0, 10.0, 3.2, 3.2, 3.2, 3.2};
  this->sample_time = sample_time; // commandPeriod [ms], minimum
  this->operation_mode = operation_mode;
  this->current_threshold = {0.00, 0.00, 200e-3, 200e-3, 200.0e-3, 200e-3}; //{600e-3, 0.00, 250e-3, 250e-3, 250e-3, 250e-3}
  // current_threshold = current_threshold * 0.0;
  this->find_limit_velocity = {0.0, 0.0, -3E-5, -3E-5, -3E-5, -3E-5}; //{600e-3, 0.00, 250e-3, 250e-3, 250e-3, 250e-3}
  // find_limit_velocity = find_limit_velocity * 0.0;

  // Define the upper and lower bounds
  this->position_lower_bounds = {-0.10, -2.0 * M_PI, -0.10, -0.10, -0.10, -0.10};
  this->position_upper_bounds = {0.10, 2.0 * M_PI, 0.10, 0.10, 0.10, 0.10};
  this->velocity_upper_bounds = {0.10, 1.0 * M_PI, 0.10, 0.10, 0.10, 0.10};
  this->velocity_lower_bounds = {-0.10, -1.0 * M_PI, -0.10, -0.10, -0.10, -0.10};

  this->posOffsets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->flag_position_limit = position_limit;

  shared_state = std::make_shared<SharedState>();

  Robot::InitializeLogger();
}

/* Copy constructor */
Robot::Robot(const Robot &rhs) : m_rotation(rhs.m_rotation),
                                 m_insertion(rhs.m_insertion),
                                 m_tendon_post(rhs.m_tendon_post),
                                 m_tendon_ant(rhs.m_tendon_ant),
                                 m_tendon_left(rhs.m_tendon_left),
                                 m_tendon_right(rhs.m_tendon_right) {};

/* class destructor: move the joints to zero position, disable, and close cotrollers. */
Robot::~Robot()
{
  logger->info("[Master]  Closing");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/* initilizes the fiber driver for each node and what the status on all nodes */
void Robot::Fiber_loop()
{
  // Initialize the I/O library. This is required on Windows, but a no-op on
  // Linux (for now).
  io::IoGuard io_guard;

  // Create an I/O context to synchronize I/O services during shutdown.
  io::Context ctx;
  // Create a platform-specific I/O polling instance to monitor the CAN bus, as
  // well as timers and signals.
  io::Poll poll(ctx);
  // Create a polling event loop and pass it the platform-independent polling
  // interface. If no tasks are pending, the event loop will poll for I/O
  // events.
  ev::Loop loop(poll.get_poll());
  // I/O devices only need access to the executor interface of the event loop.
  auto exec = loop.get_executor();
  // Create a timer using a monotonic clock, i.e., a clock that is not affected
  // by discontinuous jumps in the system time.
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);

  // Create a virtual SocketCAN CAN controller and channel, and do not modify
  // the current CAN bus state or bitrate.
  io::CanController ctrl("can0");
  io::CanChannel chan(poll, exec);

  chan.open(ctrl);

  // Create a CANopen master with node-ID 1. The master is asynchronous, which
  // means every user-defined callback for a CANopen event will be posted as a
  // task on the event loop, instead of being invoked during the event
  // processing by the stack.
  std::string master_dcf = "/master.dcf";
  std::string master_bin = "/master.bin";
  canopen::AsyncMaster master(timer, chan, CANopenFiles_directory + master_dcf, CANopenFiles_directory + master_bin, 7);

  // Create a signal handler.
  io::SignalSet sigset(poll, exec);
  // Watch for Ctrl+C or process termination.
  sigset.insert(SIGHUP);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);

  // Submit a task to be executed when a signal is raised. We don't care which.
  sigset.submit_wait([&](int /*signo*/)
                     {
    // If the signal is raised again, terminate immediately.
    sigset.clear();
    // Tell the master to start the deconfiguration process for all nodes, and
    // submit a task to be executed once that process completes.
    master.AsyncDeconfig().submit(exec, [&]() {
      // Perform a clean shutdown.
      ctx.shutdown();
    }); });

  // Start the NMT service of the master by pretending to receive a 'reset
  // node' command.
  master.Reset();

  // Run the event loop until no tasks remain (or the I/O context is shut down).

  // Create a driver for the slave with node-ID 2.
  // MyDriver driver_5(exec, master, 5);

  if (m_enable_joint[0])
  {
    exec.post([&]()
              { m_insertion = std::make_shared<CiA301Node>(exec, master, 1, "Faulhaber", encoder_res[0], gear_ratio[0], velocity_factor[0],
                                                           sample_time, operation_mode[0], max_acc[0], max_vel[0],
                                                           current_threshold[0], find_limit_velocity[0], shared_state, logger); }); // Insertion
  }

  if (m_enable_joint[1])
  {
    exec.post([&]()
              { m_rotation = std::make_shared<CiA301Node>(exec, master, 2, "Faulhaber", encoder_res[1], gear_ratio[1], velocity_factor[1],
                                                          sample_time, operation_mode[1], max_acc[1], max_vel[1],
                                                          current_threshold[1], find_limit_velocity[1], shared_state, logger); }); // Rotation
  }

  if (m_enable_joint[2])
  {
    exec.post([&]()
              { m_tendon_ant = std::make_shared<FCNode>(exec, master, 5, "Faulhaber", encoder_res[2], gear_ratio[2], velocity_factor[2],
                                                          sample_time, operation_mode[2], max_acc[2], max_vel[2],
                                                          current_threshold[2], find_limit_velocity[2], shared_state, logger); }); // Anterior
  }

  if (m_enable_joint[3])
  {
    exec.post([&]()
              { m_tendon_post = std::make_shared<FCNode>(exec, master, 6, "Faulhaber", encoder_res[3], gear_ratio[3], velocity_factor[3],
                                                        sample_time, operation_mode[3], max_acc[3], max_vel[3],
                                                        current_threshold[3], find_limit_velocity[3], shared_state, logger); }); // Posterior
  }

  if (m_enable_joint[4])
  {
    exec.post([&]()
              { m_tendon_right = std::make_shared<FCNode>(exec, master, 4, "Faulhaber", encoder_res[4], gear_ratio[4], velocity_factor[4],
                                                         sample_time, operation_mode[4], max_acc[4], max_vel[4],
                                                         current_threshold[4], find_limit_velocity[4], shared_state, logger); }); // Right
  }

  if (m_enable_joint[5])
  {
    exec.post([&]()
              { m_tendon_left = std::make_shared<FCNode>(exec, master, 3, "Faulhaber", encoder_res[5], gear_ratio[5], velocity_factor[5],
                                                         sample_time, operation_mode[5], max_acc[5], max_vel[5],
                                                         current_threshold[5], find_limit_velocity[5], shared_state, logger); }); // Left
  }

  {
    std::thread t1([&]()
                   { loop.run(); });
    t1.detach();
  }
  // this wait is mandatory to star the loop before commanding targets
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::chrono::seconds timeout(5);
  auto startTime = std::chrono::high_resolution_clock::now();
  while (!shared_state->m_boot_success)
  {

    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    if (elapsedTime >= timeout)
    {
      logger->warn("Bootup Timed Out!  --->  Reseting All Nodes -------");
      std::raise(SIGINT);
      master.Reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      // break; // Exit the loop on timeout
    }
    if ((!m_enable_joint[0] || m_insertion->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)) &&
        (!m_enable_joint[1] || m_rotation->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)) &&
        (!m_enable_joint[2] || m_tendon_post->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)) &&
        (!m_enable_joint[3] || m_tendon_ant->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)) &&
        (!m_enable_joint[4] || m_tendon_left->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)) &&
        (!m_enable_joint[5] || m_tendon_right->GetNodeFlags(Flags::FlagIndex::BOOT_SUCCESS)))
    {
      // shared_state->signalBootSuccess();
      shared_state->m_boot_success = true;
      logger->info("[Master] Nodes Booted Successfully");
    }
  }

  while (true)
  {
    if (Robot::check_all_nodes_switched_on())
    {
      if (!shared_state->m_flag_robot_switched_on)
      {
        shared_state->m_flag_robot_switched_on = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        logger->info("[Master] All Nodes Switched ON");
      }
    }
    else
    {
      if (shared_state->m_flag_robot_switched_on)
      {
        shared_state->m_flag_robot_switched_on = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        logger->info("[Master] At Least One Node Switched OFF");
      }
    }

    if (Robot::check_all_nodes_enabled())
    {
      if (!shared_state->m_flag_operation_enabled)
      {
        shared_state->m_flag_operation_enabled = true;
        shared_state->m_flag_operation_enabled_2 = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        logger->info("[Master] All Nodes Enabled");
      }
    }
    else if (Robot::check_all_nodes_disabled())
    {
      if (shared_state->m_flag_operation_enabled_2)
      {
        shared_state->m_flag_operation_enabled = false;
        shared_state->m_flag_operation_enabled_2 = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        logger->info("[Master] All Nodes Disabled");
      }
    }
    else
    {
      if (shared_state->m_flag_operation_enabled)
      {
        shared_state->m_flag_operation_enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        logger->info("[Master] At Least One Node Disabled");
      }
    }

    if (Robot::check_all_nodes_disabled())

      if (Robot::check_all_encoders_set())
      {
        if (!shared_state->m_encoders_set)
        {
          shared_state->m_encoders_set = true;
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          logger->info("[Master] Encoders Set");
        }
      }
      else
      {
        if (shared_state->m_encoders_set)
        {
          shared_state->m_encoders_set = false;
        }
      }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool Robot::check_all_nodes_switched_on()
{
  return (!m_enable_joint[0] || m_insertion->GetStatusword().switched_ON) &&
         (!m_enable_joint[1] || m_rotation->GetStatusword().switched_ON) &&
         (!m_enable_joint[2] || m_tendon_post->GetStatusword().switched_ON) &&
         (!m_enable_joint[3] || m_tendon_ant->GetStatusword().switched_ON) &&
         (!m_enable_joint[4] || m_tendon_left->GetStatusword().switched_ON) &&
         (!m_enable_joint[5] || m_tendon_right->GetStatusword().switched_ON);
}

bool Robot::check_all_nodes_enabled()
{
  return (!m_enable_joint[0] || m_insertion->GetStatusword().operation_enabled) &&
         (!m_enable_joint[1] || m_rotation->GetStatusword().operation_enabled) &&
         (!m_enable_joint[2] || m_tendon_post->GetStatusword().operation_enabled) &&
         (!m_enable_joint[3] || m_tendon_ant->GetStatusword().operation_enabled) &&
         (!m_enable_joint[4] || m_tendon_left->GetStatusword().operation_enabled) &&
         (!m_enable_joint[5] || m_tendon_right->GetStatusword().operation_enabled);
}

bool Robot::check_all_nodes_disabled()
{
  return (!m_enable_joint[0] || !m_insertion->GetStatusword().operation_enabled) &&
         (!m_enable_joint[1] || !m_rotation->GetStatusword().operation_enabled) &&
         (!m_enable_joint[2] || !m_tendon_post->GetStatusword().operation_enabled) &&
         (!m_enable_joint[3] || !m_tendon_ant->GetStatusword().operation_enabled) &&
         (!m_enable_joint[4] || !m_tendon_left->GetStatusword().operation_enabled) &&
         (!m_enable_joint[5] || !m_tendon_right->GetStatusword().operation_enabled);
}

bool Robot::check_all_encoders_set()
{
  return (!m_enable_joint[0] || m_insertion->GetNodeFlags(Flags::FlagIndex::ENCODER_SET)) &&
         (!m_enable_joint[1] || m_rotation->GetNodeFlags(Flags::FlagIndex::ENCODER_SET)) &&
         (!m_enable_joint[2] || m_tendon_post->GetNodeFlags(Flags::FlagIndex::ENCODER_SET)) &&
         (!m_enable_joint[3] || m_tendon_ant->GetNodeFlags(Flags::FlagIndex::ENCODER_SET)) &&
         (!m_enable_joint[4] || m_tendon_left->GetNodeFlags(Flags::FlagIndex::ENCODER_SET)) &&
         (!m_enable_joint[5] || m_tendon_right->GetNodeFlags(Flags::FlagIndex::ENCODER_SET));
}

/* starts the Fiber_Loop in a separate thread */
void Robot::Start_Thread()
{
  EnableThread = std::thread(&Robot::Fiber_loop, this);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while ((m_enable_joint[0] && !m_insertion->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)) ||
         (m_enable_joint[1] && !m_rotation->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)) ||
         (m_enable_joint[2] && !m_tendon_post->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)) ||
         (m_enable_joint[3] && !m_tendon_ant->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)) ||
         (m_enable_joint[4] && !m_tendon_left->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)) ||
         (m_enable_joint[5] && !m_tendon_right->GetNodeFlags(Flags::FlagIndex::TASKS_POSTED)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  logger->info("[Master] Tasks Posted");
}

/* enable operation of all joints
  @param enable: true = enable, false = disable */
void Robot::Enable_Operation(bool enable)
{
  if (m_enable_joint[0])
  {
    m_insertion->EnableOperation(enable);
  };
  if (m_enable_joint[1])
  {
    m_rotation->EnableOperation(enable);
  };
  if (m_enable_joint[2])
  {
    m_tendon_post->EnableOperation(enable);
  };
  if (m_enable_joint[3])
  {
    m_tendon_ant->EnableOperation(enable);
  };
  if (m_enable_joint[4])
  {
    m_tendon_left->EnableOperation(enable);
  };
  if (m_enable_joint[5])
  {
    m_tendon_right->EnableOperation(enable);
  };
}

/* set the current positoin of the robot as zero position on the motion controller
  - this function halts the target task loop while running */
void Robot::Set_Soft_Home(blaze::StaticVector<double, 6> offset)
{
  blaze::StaticVector<double, 6> current_pos_abs, homeOffsets;
  Robot::Get_Position_Abs(&current_pos_abs);

  homeOffsets = current_pos_abs - offset;

  m_insertion->SetHomeOffsetValue(homeOffsets[0]);
  m_rotation->SetHomeOffsetValue(homeOffsets[1]);
  m_tendon_ant->SetHomeOffsetValue(homeOffsets[2]);
  m_tendon_post->SetHomeOffsetValue(homeOffsets[3]);
  m_tendon_right->SetHomeOffsetValue(homeOffsets[4]);
  m_tendon_left->SetHomeOffsetValue(homeOffsets[5]);
  
  logger->info("[Master] Setting soft home done");
}

/* set the current positoin of the robot as zero position on the motion controller
  - this function halts the target task loop while running */
void Robot::Set_Zero_Position(blaze::StaticVector<double, 6> offset)
{
  logger->info("[Master] Setting zero position");

  m_insertion->m_isConfiguring = true;
  m_rotation->m_isConfiguring = true;
  m_tendon_ant->m_isConfiguring = true;
  m_tendon_post->m_isConfiguring = true;
  m_tendon_right->m_isConfiguring = true;
  m_tendon_left->m_isConfiguring = true;

  while (m_insertion->m_isConfiguring || m_rotation->m_isConfiguring || m_tendon_post->m_isConfiguring ||
         m_tendon_ant->m_isConfiguring)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  m_insertion->SetHomeOffsetValue(offset[0]);
  m_rotation->SetHomeOffsetValue(offset[1]);
  m_tendon_ant->SetHomeOffsetValue(offset[2]);
  m_tendon_post->SetHomeOffsetValue(offset[3]);
  m_tendon_right->SetHomeOffsetValue(offset[4]);
  m_tendon_left->SetHomeOffsetValue(offset[5]);

  logger->info("[Master] Setting zero position done");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/* find the positive mechanical limit of the linear joints based on current threshold
  - this function currently works when the mode of operation is PV*/
void Robot::Find_Fowrard_Limits()
{
  blaze::StaticVector<double, 6UL> vel_findlimit = blaze::StaticVector<double, 6>(0.0); //[mm] or [deg] / [sec]
  blaze::StaticVector<double, 6UL> current_threshold = {600, 000, 300, 300, 300, 300};  // [mA]
  blaze::StaticVector<double, 6UL> p_current = blaze::StaticVector<double, 6>(0.0);
  blaze::StaticVector<bool, 6UL> limit_reached = {!m_enable_joint[0], !m_enable_joint[1], !m_enable_joint[2], !m_enable_joint[3], !m_enable_joint[4], !m_enable_joint[5]};

  const int maxBufferSize = 100;
  std::vector<std::deque<int>> currentBuffers(6); // Circular buffers for each joint
  std::vector<double> current_avg(6, 0.0);

  m_insertion->m_isConfiguring = true;
  m_insertion->SetOperationMode(OpMode::VelocityProfile);
  m_insertion->m_isConfiguring = false;

  this->Set_Target_Velocity(vel_findlimit);

  if (m_enable_joint[0])
  {
    m_insertion->EnableOperation(true);
  };
  if (m_enable_joint[1])
  {
    m_rotation->EnableOperation(true);
  };
  if (m_enable_joint[2])
  {
    m_tendon_post->EnableOperation(true);
  };
  if (m_enable_joint[3])
  {
    m_tendon_ant->EnableOperation(true);
  };
  if (m_enable_joint[4])
  {
    m_tendon_left->EnableOperation(true);
  };
  if (m_enable_joint[5])
  {
    m_tendon_right->EnableOperation(true);
  };

  while (!std::all_of(limit_reached.begin(), limit_reached.end(), [](bool l)
                      { return l; }))
  {
    // update average current
    this->Get_Current(&p_current);
    for (int node = 0; node < 6; ++node)
    {
      currentBuffers[node].push_back(p_current[node]);

      if (currentBuffers[node].size() > maxBufferSize)
      {
        currentBuffers[node].pop_front(); // Keep the buffer size at 100
      }

      int sum = 0;
      for (int current : currentBuffers[node])
      {
        sum += current;
      }

      current_avg[node] = static_cast<double>(sum) / currentBuffers[node].size();
    }

    if (abs(current_avg[0]) >= current_threshold[0] && !limit_reached[0])
    {
      limit_reached[0] = true;
      vel_findlimit[0] = 0;
      m_insertion->EnableOperation(false);
      m_insertion->m_isConfiguring = true;
    }

    if (abs(current_avg[1]) >= current_threshold[1] && !limit_reached[1])
    {
      limit_reached[1] = true;
      vel_findlimit[1] = 0;
      m_rotation->EnableOperation(false);
      m_rotation->m_isConfiguring = true;
    }

    if (abs(current_avg[2]) >= current_threshold[2] && !limit_reached[2])
    {
      limit_reached[2] = true;
      vel_findlimit[2] = 0;
      m_tendon_post->EnableOperation(false);
      m_tendon_post->m_isConfiguring = true;
    }

    if (abs(current_avg[3]) >= current_threshold[3] && !limit_reached[3])
    {
      limit_reached[3] = true;
      vel_findlimit[3] = 0;
      m_tendon_ant->EnableOperation(false);
      m_tendon_ant->m_isConfiguring = true;
    }

    if (abs(current_avg[4]) >= current_threshold[4] && !limit_reached[4])
    {
      limit_reached[4] = true;
      vel_findlimit[4] = 0;
      m_tendon_left->EnableOperation(false);
      m_tendon_left->m_isConfiguring = true;
    }

    if (abs(current_avg[5]) >= current_threshold[5] && !limit_reached[5])
    {
      limit_reached[5] = true;
      vel_findlimit[5] = 0;
      m_tendon_right->EnableOperation(false);
      m_tendon_right->m_isConfiguring = true;
    }

    std::cout << "Finding limit... \n"
              << "Node 1:"
              << "averaged current :" << current_avg[0] << " | reached: " << limit_reached[0] << "\n"
              << "Node 2:"
              << "averaged current :" << current_avg[1] << " | reached: " << limit_reached[1] << "\n"
              << "Node 3:"
              << "averaged current :" << current_avg[2] << " | reached: " << limit_reached[2] << "\n"
              << "Node 4:"
              << "averaged current :" << current_avg[3] << " | reached: " << limit_reached[3] << "\n"
              << "Node 5:"
              << "averaged current :" << current_avg[4] << " | reached: " << limit_reached[4] << "\n"
              << "Node 6:"
              << "averaged current :" << current_avg[5] << " | reached: " << limit_reached[5] << "\n"
              << "--------------------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
  Robot::Set_Target_Velocity(v);
  Robot::Enable_Operation(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "---- Finding limit done ----" << std::endl;
}

/* returns the status of motion controller switches
  - true, when all motion controllers are switched on and ready to enable operation */
bool Robot::Get_Controller_Switch_Status()
{
  return shared_state->m_flag_robot_switched_on;
}

/* Actuatre robot to the targert absolute (with respect to the 0 (forward limit) position) joint positions in mm or deg unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void Robot::Set_Target_Position_Abs(blaze::StaticVector<double, 6> posTarget)
{
  if (this->flag_position_limit)
  {
    if (!(Robot::Position_limits_check(posTarget) == 0))
    {
      return;
    }
  }

  if (m_enable_joint[0])
  {
    m_insertion->SetTargetPosAbs(posTarget[0]); // in [mm]
  }
  if (m_enable_joint[1])
  {
    m_rotation->SetTargetPosAbs(posTarget[1]); // in [deg]
  }
  if (m_enable_joint[2])
  {
    m_tendon_ant->SetTargetPosAbs(posTarget[2]); // in [mm]
  }
  if (m_enable_joint[3])
  {
    m_tendon_post->SetTargetPosAbs(posTarget[3]); // in [mm]
  }
  if (m_enable_joint[4])
  {
    m_tendon_right->SetTargetPosAbs(posTarget[4]); // in [mm]
  }
  if (m_enable_joint[5])
  {
    m_tendon_left->SetTargetPosAbs(posTarget[5]); // in [mm]
  }
}

/* Actuatre robot to the targert joint positions in mm or deg unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void Robot::Set_Target_Position(blaze::StaticVector<double, 6> posTarget)
{
  if (this->flag_position_limit)
  {
    if (!(Robot::Position_limits_check(posTarget) == 0))
    {
      return;
    }
  }

  if (m_enable_joint[0])
  {
    m_insertion->SetTargetPos(posTarget[0]); // in [mm]
  }
  if (m_enable_joint[1])
  {
    m_rotation->SetTargetPos(posTarget[1]); // in [deg]
  }
  if (m_enable_joint[2])
  {
    m_tendon_ant->SetTargetPos(posTarget[2]); // in [mm]
  }
  if (m_enable_joint[3])
  {
    m_tendon_post->SetTargetPos(posTarget[3]); // in [mm]
  }
  if (m_enable_joint[4])
  {
    m_tendon_right->SetTargetPos(posTarget[4]); // in [mm]
  }
  if (m_enable_joint[5])
  {
    m_tendon_left->SetTargetPos(posTarget[5]); // in [mm]
  }
}

/* Actuatre robot to the targert absolute joint positions in [mm/s] or [deg/s] unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void Robot::Set_Target_Velocity(blaze::StaticVector<double, 6> velTarget)
{

  // Clamp the velTarget values to stay within the bounds
  for (size_t i = 0; i < velTarget.size(); ++i)
  {
    if (velTarget[i] > velocity_upper_bounds[i])
    {
      velTarget[i] = velocity_upper_bounds[i];
    }
    else if (velTarget[i] < velocity_lower_bounds[i])
    {
      velTarget[i] = velocity_lower_bounds[i];
    }
  }

  // Now set the clamped values

  if (m_enable_joint[0])
  {
    m_insertion->SetTargetVel(velTarget[0]); // in [mm]
  }
  if (m_enable_joint[1])
  {
    m_rotation->SetTargetVel(velTarget[1]); // in [deg]
  }
  if (m_enable_joint[2])
  {
    m_tendon_ant->SetTargetVel(velTarget[2]); // in [mm]
  }
  if (m_enable_joint[3])
  {
    m_tendon_post->SetTargetVel(velTarget[3]); // in [mm]
  }
  if (m_enable_joint[4])
  {
    m_tendon_right->SetTargetVel(velTarget[4]); // in [mm]
  }
  if (m_enable_joint[5])
  {
    m_tendon_left->SetTargetVel(velTarget[5]); // in [mm]
  }
}

/* Gets the motor current in [mA] unit */
void Robot::Get_Current(blaze::StaticVector<double, 6> *p_current)
{
  if (p_current)
  {

    if (m_enable_joint[0])
    {
      this->m_insertion->GetCurrent(((*p_current)[0]));
    }
    if (m_enable_joint[1])
    {
      this->m_rotation->GetCurrent(((*p_current)[1]));
    }
    if (m_enable_joint[2])
    {
      this->m_tendon_ant->GetCurrent(((*p_current)[2]));
    }
    if (m_enable_joint[3])
    {
      this->m_tendon_post->GetCurrent(((*p_current)[3]));
    }
    if (m_enable_joint[4])
    {
      this->m_tendon_right->GetCurrent(((*p_current)[4]));
    }
    if (m_enable_joint[5])
    {
      this->m_tendon_left->GetCurrent(((*p_current)[5]));
    }
  }
}

/* Gets the current velocity (with respect to zero position - distal limit) of all actuators in [mm/s] or [deg/s] unit */
void Robot::Get_Velocity(blaze::StaticVector<double, 6> *p_velCurrent)
{
  if (p_velCurrent)
  {

    if (m_enable_joint[0])
    {
      this->m_insertion->GetActualVel(((*p_velCurrent)[0]));
    }
    if (m_enable_joint[1])
    {
      this->m_rotation->GetActualVel(((*p_velCurrent)[1]));
    }
    if (m_enable_joint[2])
    {
      this->m_tendon_ant->GetActualVel(((*p_velCurrent)[2]));
    }
    if (m_enable_joint[3])
    {
      this->m_tendon_post->GetActualVel(((*p_velCurrent)[3]));
    }
    if (m_enable_joint[4])
    {
      this->m_tendon_right->GetActualVel(((*p_velCurrent)[4]));
    }
    if (m_enable_joint[5])
    {
      this->m_tendon_left->GetActualVel(((*p_velCurrent)[5]));
    }
  }
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void Robot::Get_Position_Abs(blaze::StaticVector<double, 6> *p_posCurrent)
{
  if (p_posCurrent)
  {
    if (m_enable_joint[0])
    {
      this->m_insertion->GetActualPosAbs(((*p_posCurrent)[0]));
    }
    if (m_enable_joint[1])
    {
      this->m_rotation->GetActualPosAbs(((*p_posCurrent)[1]));
    }
    if (m_enable_joint[2])
    {
      this->m_tendon_ant->GetActualPosAbs(((*p_posCurrent)[2]));
    }
    if (m_enable_joint[3])
    {
      this->m_tendon_post->GetActualPosAbs(((*p_posCurrent)[3]));
    }
    if (m_enable_joint[4])
    {
      this->m_tendon_right->GetActualPosAbs(((*p_posCurrent)[4]));
    }
    if (m_enable_joint[5])
    {
      this->m_tendon_left->GetActualPosAbs(((*p_posCurrent)[5]));
    }
  }
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void Robot::Get_Position(blaze::StaticVector<double, 6> *p_posCurrent)
{
  if (p_posCurrent)
  {
    if (m_enable_joint[0])
    {
      this->m_insertion->GetActualPos(((*p_posCurrent)[0]));
    }
    if (m_enable_joint[1])
    {
      this->m_rotation->GetActualPos(((*p_posCurrent)[1]));
    }
    if (m_enable_joint[2])
    {
      this->m_tendon_ant->GetActualPos(((*p_posCurrent)[2]));
    }
    if (m_enable_joint[3])
    {
      this->m_tendon_post->GetActualPos(((*p_posCurrent)[3]));
    }
    if (m_enable_joint[4])
    {
      this->m_tendon_right->GetActualPos(((*p_posCurrent)[4]));
    }
    if (m_enable_joint[5])
    {
      this->m_tendon_left->GetActualPos(((*p_posCurrent)[5]));
    }
  }
}

/**/
void Robot::Get_PosVelCur(blaze::StaticVector<double, 6> *p_posCurrent_abs,
                          blaze::StaticVector<double, 6> *p_posCurrent,
                          blaze::StaticVector<double, 6> *p_velCurrent,
                          blaze::StaticVector<double, 6> *p_current)
{
  Robot::Get_Position_Abs(p_posCurrent_abs);
  Robot::Get_Position(p_posCurrent);
  Robot::Get_Velocity(p_velCurrent);
  Robot::Get_Current(p_current);
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void Robot::Convert_pos_to_CTR_frame(blaze::StaticVector<double, 6> &posCurrent,
                                     blaze::StaticVector<double, 6> *posInCTRFrame)
{
  for (size_t i = 0; i < posCurrent.size(); ++i)
  {
    (*posInCTRFrame)[i] = posCurrent[i] + this->posOffsets[i];
  }
}

/**/
int Robot::Position_limits_check(blaze::StaticVector<double, 6> posTarget)
{
  for (size_t i = 0; i < posTarget.size(); ++i)
  {
    if (posTarget[i] < position_lower_bounds[i])
    {
      std::cout << "static lower bound reached" << std::endl;
      return -1;
    }
    if (posTarget[i] > position_upper_bounds[i])
    {
      std::cout << "static upper bound reached" << std::endl;
      return -1;
    }
  }
  blaze::StaticVector<double, 6> posInCTRFrame = blaze::StaticVector<double, 6>(0);

  Robot::Convert_pos_to_CTR_frame(posTarget, &posInCTRFrame);

  return 0;
}

/**/
bool Robot::Get_reachStatus()
{
  if (m_rotation->IsReached() && m_insertion->IsReached() &&
      m_tendon_post->IsReached() && m_tendon_ant->IsReached() &&
      m_tendon_left->IsReached() && m_tendon_right->IsReached())
  {
    return 1;
  }
  else
  {
    return 0;
  }
  // std::vector<double> thresholds = {0.1, 0.05, 0.1, 0.05};
  // std::vector<double> velocity;
  // CatheterRobot::Get_Velocity(&velocity);

  // for (size_t i = 0; i < velocity.size(); i++)
  // {
  //   if (std::abs(velocity[i]) >= thresholds[i])
  //   {
  //     return false; // At least one velocity exceeds the threshold
  //   }
  // }
  // return true;
}

/**/
void Robot::Wait_until_reach()
{
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!Robot::Get_reachStatus())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/* Get the current date and time and format the filename using the date and time  */
void Robot::InitializeLogger()
{
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  struct tm tm;
  localtime_r(&time, &tm);
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &tm);
  std::string filename = std::string(buffer) + ".txt"; // Use the formatted time as the filename
  std::filesystem::create_directories(Log_directory);
  std::string logDir = Log_directory + filename;

  /* instansiate the logger object */
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logDir, true);
  console_sink->set_level(spdlog::level::info);                                                                // Only print warnings and above to console
  file_sink->set_level(spdlog::level::debug);                                                                  // Log all messages to the file
  this->logger = std::make_shared<spdlog::logger>("ICEBot", spdlog::sinks_init_list{console_sink, file_sink}); // instansiate the logger
  this->logger->set_level(spdlog::level::debug);                                                               // This enables debug messages to be processed
  this->logger->flush_on(spdlog::level::debug);                                                                // Flush immediately on debug level
}
