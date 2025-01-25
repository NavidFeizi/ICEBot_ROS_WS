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

/*
  This class is responsible for creating virtual nodes that facilitate communication with
  motion controllers (Faulhaber or Maxon) using the CANopen protocol.
  It supports the CiA301 operation layer, Faulhaber channel commands (which is now considered obsolete), and CiA402 device profile .
*/

#pragma once

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/driver.hpp>
#include <lely/coapp/device.hpp>
#include <lely/coapp/node.hpp>
#include <lely/coapp/master.hpp>
#include <lely/util/result.hpp>
#include <lely/ev/future.hpp>
#include <iostream>
#include <thread>
#include <string>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <memory>
#include "spdlog/spdlog.h"

#include "CiA301node.hpp"
#include "CatheterRobot.hpp"
#include "CanEssentials.hpp"
#include "SharedStates.hpp"

using namespace std::chrono_literals;
using namespace lely;

class Flags;

class CiA301Node : public canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;
  // Constructor and Destructor
  CiA301Node(ev_exec_t *exec, canopen::AsyncMaster &master, unsigned int id, std::string ControllerBrand, double encoderRes,
             double gearRatio,  double VelocityFactor, unsigned int commandPeriod, OpMode operationMode, double maxAcc,
             double maxVel, double current_threshold, double vel_findlimit, std::shared_ptr<SharedState> sharedState,
             std::shared_ptr<spdlog::logger> shared_logger);
  ~CiA301Node();

  // ============================== Command Methods ==============================
  void SetTargetPosAbs(const double val);
  void SetTargetPos(const double val);
  void SetTargetVel(const double val);
  void BackToZero();
  void EnableOperation(const bool enable);

  // ============================== Configuration Methods ==============================
  void SetOperationMode(const OpMode operationMode);
  void SetHomeOffsetValue(const double val);
  StatusWord GetStatusword() const;
  bool GetNodeFlags(Flags::FlagIndex index) const;

  // ============================== Feedback Methods ==============================
  void GetCurrent(double &motorCurrentPtr) const;
  void GetActualPosAbs(double &actualPosPtr) const;
  void GetActualPos(double &actualPosPtr) const;
  void GetActualVel(double &actualVelPtr) const;
  bool IsReached() const;

private:
  // ============================== CANopen Communication Methods ==============================
  void OnBoot(canopen::NmtState st, char es, const std::string &what) noexcept override;
  void OnConfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnSync(uint8_t cnt, const time_point &t) noexcept override;
  void TaskTarget() noexcept;
  void TaskConfig() noexcept;

  // ============================== CiA402 State Machine Methods ==============================
  void SwitchOn();
  void EnableOperationWithPdo(const bool enable);
  void ResetFault();

  // ============================== Configuration Methods ==============================
  void SetTpdoTranstype(const int numPDO, const int type);
  void SetProfileParams(const int maxACC, const int maxDCC, const int maxSpeed);
  void SetEncoder(const double offset);
  void FindFowrardLimits();

  // ============================== Utility Methods ==============================
  void ReadEncoderMemFile(const std::string &directory, double &encoder_memory, std::ofstream &encoder_memory_file);

public:
  int m_isConfiguring = 0; // 1=set zero, 2=find limit

private:
  // ============================== Physical constants ==============================
  std::string m_node_id;          // node ID string to facilitate logging
  std::string m_controller_brand; // MotionController brand -- "Maxon" or "Faulhaber"
  double m_ppu;                   // pulse per unit (user defined unit for Faulhaber)
  double m_gear_ratio;            // rad->rev or m->rev
  double m_conversion_factor;
  int m_axis_dir;

  // ============================ Configurable variables ============================
  unsigned int m_sample_time;   // unit: [ms]
  int m_profile_acc;         // in SI unit
  int m_profile_vel;         // in SI unit  (only for position profile (PP) mode)
  double m_vel_findlimit;       // in SI unit
  double m_current_threshold;   // in mA or ??
  double m_pos_offset_SI = 0.0; // in SI unit
  OpMode m_operation_mode;
  OpMode m_current_operation_mode;

  // ======================== Command and feedback variables ========================
  double m_target_pos_SI;    // in SI unit
  double m_target_vel_SI;    // in SI unit
  double m_actual_pos_SI;    // in SI unit
  double m_actual_vel_SI;    // in SI unit
  double m_current_SI;       // in SI unit
  int32_t m_target_pos;      // in motion controller unit (pulse/) not in user defined unit
  int32_t m_target_pos_prev; // in motion controller unit (pulse/) not in user defined unit
  int32_t m_target_vel;      // in motion controller unit (pulse/) not in user defined unit
  int32_t m_actual_pos;      // in motion controller unit (pulse/) not in user defined unit
  int32_t m_actual_vel;      // in motion controller unit (pulse/) not in user defined unit
  int16_t m_current;         // current(Maxon) ot torque(Faulhaber)

  // ======================== Other variables ========================
  Flags m_flags;                             // node state flags
  std::shared_ptr<SharedState> robot_states; // Shared all node states set by the master (Robot)
  std::shared_ptr<spdlog::logger> logger;    // Shared logger instance
  ControlWord m_control_word;                // Control words updates automatically on RPDO1 write
  StatusWord m_status_word;                  // Status words updates automatically on RPDO1 write
  std::ofstream m_encoder_memory_file;       // File to read from / write on encoder memory to keep track of positoin for future start ups
  double m_encoder_memory_value;             // Loaded previous encoder value from memory file
  bool m_print_pdos = false;                 // for tracing debug
  bool bit12_prev = false;
};
