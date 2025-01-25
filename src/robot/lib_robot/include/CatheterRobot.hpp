//==========================================================================================================
// 					 This code is a part of the robot actuated catheter project
//  		   		Canadian Surgical Technologies and Advanced Robotics (CSTAR).
// 							Copyright (C) 2022 Navid Feizi <nfeizi@uwo.ca>
//					     Copyright (C) 2022 Filipe C. Pedrosa <fpedrosa@uwo.ca>
//
//       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)
//  ========================================================================================================
//   (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada
//             (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School
//==========================================================================================================

#pragma once

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/util/result.hpp>
#include <iostream>
#include <lely/ev/future.hpp>
#include <thread>
#include <string>
#include <bitset>
#include <fstream>
#include <cmath>
#include <future>
#include <deque>
#include <filesystem>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

#include "CiA301node.hpp"
#include "FCnode.hpp"
#include "CatheterRobot.hpp"
#include "SharedStates.hpp"

#include <cstring>
#include <csignal>

#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#define Sleep(x) usleep((x) * 1000)
#endif

#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>

class CiA301Node;
class FCNode;

class Robot
{

public:
    Robot(int sample_time, blaze::StaticVector<OpMode, 6UL> operation_mode, blaze::StaticVector<double, 6UL> max_acc,
          blaze::StaticVector<double, 6UL> max_vel, bool position_limit);
    Robot(const Robot &rhs);
    ~Robot();
    void initialize_io();
    bool Get_Controller_Switch_Status();
    void Start_Thread();
    void Enable_Operation(const bool enable);
    void Set_Zero_Position(const blaze::StaticVector<double, 6> offset);
    void Set_Soft_Home(const blaze::StaticVector<double, 6> offset);
    void Find_Fowrard_Limits();
    void Set_Target_Position_Abs(const blaze::StaticVector<double, 6> &posTarget);
    void Set_Target_Position(const blaze::StaticVector<double, 6> &posTarget);
    void Set_Target_Velocity(const blaze::StaticVector<double, 6> &velTarget);
    void Get_Current(blaze::StaticVector<double, 6> &current);
    void Get_Velocity(blaze::StaticVector<double, 6> &velCurrent);
    void Get_Position_Abs(blaze::StaticVector<double, 6> &posCurrent);
    void Get_Position(blaze::StaticVector<double, 6> &posCurrent);
    void Get_PosVelCur(blaze::StaticVector<double, 6> &posCurrent_abs,
                       blaze::StaticVector<double, 6> &posCurrent,
                       blaze::StaticVector<double, 6> &velCurrent,
                       blaze::StaticVector<double, 6> &current);
    bool Get_reachStatus();
    void Wait_until_reach();

    bool check_all_nodes_switched_on();
    bool check_all_nodes_enabled();
    bool check_all_nodes_disabled();
    bool check_all_encoders_set();

    // bool m_boot_success = false;
    // bool m_flag_robot_switched_on = false;
    // bool m_flag_operation_enabled, m_flag_operation_enabled_2 = false;
    // bool m_encoders_set;

private:
    void Fiber_loop();
    void Convert_pos_to_CTR_frame(const blaze::StaticVector<double, 6> &posCurrent, blaze::StaticVector<double, 6> &posInCTRFrame);
    int Position_limits_check(blaze::StaticVector<double, 6> posTarget);
    void InitializeLogger();

    std::shared_ptr<CiA301Node> m_rotation;
    std::shared_ptr<CiA301Node> m_insertion;
    std::shared_ptr<FCNode> m_tendon_post;
    std::shared_ptr<FCNode> m_tendon_ant;
    std::shared_ptr<FCNode> m_tendon_left;
    std::shared_ptr<FCNode> m_tendon_right;
    std::thread EnableThread;

    std::shared_ptr<spdlog::logger> logger;

    std::shared_ptr<SharedState> shared_state;

    unsigned int sample_time; // commandPeriod [ms], minimum
    blaze::StaticVector<OpMode, 6UL> operation_mode;
    std::vector<double> encoder_res;
    std::vector<double> velocity_factor;
    std::vector<double> gear_ratio;
    blaze::StaticVector<double, 6UL> max_acc; // deg->rev or mm->rev
    blaze::StaticVector<double, 6UL> max_vel;
    blaze::StaticVector<double, 6UL> current_threshold;
    blaze::StaticVector<double, 6UL> find_limit_velocity;
    blaze::StaticVector<bool, 6UL> m_enable_joint;

    blaze::StaticVector<double, 6UL> position_lower_bounds; // SI
    blaze::StaticVector<double, 6UL> position_upper_bounds; // SI
    blaze::StaticVector<double, 6UL> velocity_lower_bounds; // SI
    blaze::StaticVector<double, 6UL> velocity_upper_bounds; // SI

    blaze::StaticVector<double, 6UL> posOffsets;
    double clearance_min;
    double clearance_max;
    bool flag_position_limit;

protected:
};
