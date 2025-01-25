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
  This class is responsible for creating nodes that facilitate communication with motion controllers (Faulhaber
  and Maxon) using the CANopen protocol. It supports the CiA301 operation layer, and CiA402 device profile .
*/

#include "CiA301node.hpp"

using namespace std::chrono_literals;
using namespace lely;

extern std::unordered_map<std::string, uint8_t> faulhaberComCodeDictionary;

/** Overloaded Constructor
 * @brief FiberDriver base class with master and id parameters
 *
 * @param parent Pointer to the parent robot class. Used for log record and some flag check.
 * @param exec Pointer to an event executor (ev_exec_t) for asynchronous operations.
 * @param master A reference to a CANopen AsyncMaster, which is responsible for communication.
 * @param NodeID The unique node ID (1-256) of the motion controller.
 * @param EncoderResolution Pulse per SI position unit.
 * @param GearRatio Gear ratio if not set in the motion controller factors.
 * @param VelocityFactor The velocity factor multiplier set in the motion controller factors / 60.
 * @param SampleTime An unsigned integer specifying the command execution period in [ms] for this Node.
 * @param OperationMode An int that defines the mode of operation (position profile, velocity profile, etc.).
 * @param ProfileAcc Profile position or profile velocity acceleration in SI units.
 * @param ProfileVel Profile position velocity in SI units.
 */
CiA301Node::CiA301Node(ev_exec_t * /*exec*/,
                       canopen::AsyncMaster &master,
                       unsigned int NodeID,
                       std::string ControllerBrand,
                       double EncoderResolution,
                       double GearRatio,
                       double VelocityFactor,
                       unsigned int SampleTime,
                       OpMode OperationMode,
                       double ProfileAccSI,
                       double ProfileVelSI,
                       double CurrentThreshold,
                       double VelFindLimit,
                       std::shared_ptr<SharedState> sharedState,
                       std::shared_ptr<spdlog::logger> shared_logger)
    : FiberDriver(master, NodeID), robot_states(sharedState), logger(shared_logger)
{
    this->m_controller_brand = ControllerBrand;
    this->m_sample_time = SampleTime;
    this->m_operation_mode = OperationMode;
    this->m_gear_ratio = GearRatio;
    this->m_ppu = (EncoderResolution / GearRatio);
    this->m_current_threshold = CurrentThreshold;
    this->m_vel_findlimit = VelFindLimit;
    this->m_node_id = std::to_string(NodeID);

    m_flags.set(Flags::FlagIndex::BOOT_SUCCESS, false);
    m_flags.set(Flags::FlagIndex::TASKS_POSTED, false);
    m_flags.set(Flags::FlagIndex::ENCODER_SET, false);
    m_flags.set(Flags::FlagIndex::NEW_TARG_READY, false);
    m_flags.set(Flags::FlagIndex::ENCODER_MEM_READY, false);

    // check motion controller brand and convert profile position parameters
    if (m_controller_brand == "Maxon" || m_controller_brand == "Faulhaber")
    {
        m_conversion_factor = (m_controller_brand == "Maxon") ? (60.0 / m_gear_ratio) : (m_ppu / VelocityFactor);
        m_profile_vel = static_cast<unsigned int>(abs(ProfileVelSI * m_conversion_factor));
        m_profile_acc = static_cast<unsigned int>(abs(ProfileAccSI * m_conversion_factor));
    }
    else
    {
        logger->critical("[Node " + m_node_id + "] Unknown motion controller brand | \"Maxon\", and \"Faulhaber\" are supported.");
    }

    logger->debug("[Node " + m_node_id + "] " +
                  "Controller brand: \"" + m_controller_brand + "\" - " +
                  "Operation layer: " + " CiA301 - " +
                  "PPU: " + std::to_string(m_ppu) + " - " +
                  "Gear ratio: " + std::to_string(GearRatio) + " - " +
                  "Encoder resoluton: " + std::to_string(EncoderResolution));
}

CiA301Node::~CiA301Node()
{
    // if (m_encoder_memory_file.is_open())
    //     m_encoder_memory_file.close();
}

// ============================== CANopen Communication Methods ==============================
/* This function gets called during the boot-up process for the node. The
    'res' parameter is the function that MUST be invoked when the configuration
    is complete. Because this function runs as a task inside a coroutine, it
    can suspend itself and wait for an asynchronous function, such as an SDO
    request, to complete. */
void CiA301Node::OnConfig(std::function<void(std::error_code ec)> res) noexcept
{
    logger->debug("[Node " + m_node_id + "] Configing");
    Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
    try
    {
        res({}); // Indicate successful completion of the configuration by invoking the 'res' function.
    }
    catch (canopen::SdoError &e)
    {
        // If one of the SDO requests resulted in an error, abort the configuration
        // and report the error code using the 'res' function.
        logger->debug("[Node " + m_node_id + "] Error Configing");
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        res(e.code());
    }
}

/* This function gets called when the boot-up process of the node completes.
    The 'st' parameter contains the last known NMT state of the slave
    (typically pre-operational), 'es' the error code (0 on success), and 'what'
    a description of the error, if any.*/
void CiA301Node::OnBoot(canopen::NmtState /*st*/, char es, const std::string &what) noexcept
{
    if (!es || es == 'L')
    {
        // Successful boot-up or boot-up is in progress ("L" state).
        m_flags.set(Flags::FlagIndex::BOOT_SUCCESS, true);
        logger->debug("[Node " + m_node_id + "] Booted successfully.");

        while (!robot_states->m_boot_success)                         // wait for all other nodes to boot
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for all other nodes to boot

        CiA301Node::ResetFault();                                     // reset faults if there is any
        CiA301Node::SwitchOn();                                       // switch ON
        while (!robot_states->m_flag_robot_switched_on)               // wait for other nodes to switch ON
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to switch ON
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));    // wait for better log

        CiA301Node::SetOperationMode(m_operation_mode);                                     // set mode of operation
        CiA301Node::SetProfileParams(m_profile_acc, m_profile_acc, m_profile_vel); // set profile parameters
        Wait(AsyncWait(duration(std::chrono::milliseconds(200))));                          // wait for for a clean log file

        CiA301Node::SetTpdoTranstype(1, 1);                       // set tPDO1 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        CiA301Node::SetTpdoTranstype(2, 1);                       // set tPDO2 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file

        // CiA301Node::ReadEncoderMemFile(EnoderStoreFiles_directory,
        //                                m_encoder_memory_value,
        //                                m_encoder_memory_file);     // read or create the memory file
        Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // this wait is necessary to give time opening memory files
        m_flags.set(Flags::FlagIndex::ENCODER_MEM_READY, true);

        // Node::Find_Fowrard_Limits();
        // CiA301Node::SetEncoder(m_encoder_memory_value);               // set encoder value
        CiA301Node::EnableOperation(true);                            // enable operation before posting the tasks
        while (!robot_states->m_flag_operation_enabled)               // wait for other nodes to enabled
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to enabled
        Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));   // wait for a clean log
        m_flags.set(Flags::FlagIndex::NEW_TARG_READY, true);

        Post(&CiA301Node::TaskTarget, this);
        Post(&CiA301Node::TaskConfig, this);
        m_flags.set(Flags::FlagIndex::TASKS_POSTED, true);
        logger->debug("[Node " + m_node_id + "] Task posted");
    }
    else
    {
        logger->critical("[Node " + m_node_id + "] Failed to boot: " + what);
    }
}

/* This function is similar to OnConfg(), but it gets called by the
    AsyncDeconfig() method of the master. */
void CiA301Node::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept
{
    logger->debug("[Node " + m_node_id + "] Deconfiging");
    try
    {
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 0;                                            // set quick stop
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));    // write on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));                 // wait for safety
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word
        // if (m_encoder_memory_file.is_open())                                      // close encoder memory file
        //     m_encoder_memory_file.close();
        logger->debug("[Node " + m_node_id + "] " + m_status_word.getCiA402StatusMessage());
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        res({});
    }
    catch (canopen::SdoError &e)
    {
        logger->error("[Node " + m_node_id + "] Error Deconfiging");
        res(e.code());
    }
}

/* This function gets called every time a value is written to the local object
    dictionary of the master by an RPDO (or SDO, but that is unlikely for a
    master), *and* the object has a known mapping to an object on the slave for
    which this class is the driver. The 'idx' and 'subidx' parameters are the
    object index and sub-index of the object on the slave, not the local object
    dictionary of the master. */
void CiA301Node::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
    // if RxPDO 1 received ---- info in RxPDO1 -> [1] Status Word
    if (idx == STATUS_WORD_IDX && subidx == 0)
    {
        m_status_word.update(rpdo_mapped[STATUS_WORD_IDX][0]); // update status word

        // check the target position acknowledge bit (12-stw) => setpoint is acknowlesged by the motioncontroller
        // switch off the new target bit (4-ctrlw) if acknowledged
        if (m_current_operation_mode == OpMode::PositionProfile)
        {
            if (m_status_word.bit12 && !bit12_prev) // rising edge of bit12 (set point acknowledge)
            {
                m_control_word.bit4 = 0;                                 // set new target bit (bit 4)
                tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // unset new target bit
                tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();           // Trigger write events for PDO1.
                bit12_prev = m_status_word.bit12;
                if (m_print_pdos)
                {
                    std::cout << "=> tPDO1 [" << m_node_id << "]: Control: 0b" << ToBinaryString(m_control_word.get()) << std::endl;
                    std::cout << "         [" << m_node_id << "]:[" << std::hex << CONTROL_WORD_IDX << "][" << static_cast<int>(0) << "]: 0x" << std::hex << m_control_word.get() << std::endl;
                }
            }
            else if (!m_status_word.bit12 && bit12_prev) // falling edge of bit12 (set point acknowledge)
            {
                m_flags.set(Flags::FlagIndex::NEW_TARG_READY, true);
                bit12_prev = m_status_word.bit12;
            }
        }
    }
    // if RxPDO 1 received ---- info in RxPDO1 -> [2] Actual Positon
    if (idx == ACTUAL_POSITION_IDX && subidx == 0)
    {
        m_actual_pos = rpdo_mapped[ACTUAL_POSITION_IDX][0];     // update actual position
        m_actual_pos_SI = double(m_actual_pos) / double(m_ppu); // convert to SI unit

        // // store the actual posotion in encoder memory  // disables for the ICEBot
        // if (m_status_word.switched_ON && m_flags.get(Flags::FlagIndex::ENCODER_MEM_READY))
        // {
        //     if (m_encoder_memory_file.is_open())
        //     {
        //         m_encoder_memory_file.seekp(0); // Move the write pointer to the beginning
        //         double pval;
        //         CiA301Node::GetActualPos(pval);
        //         m_encoder_memory_file << std::fixed << std::setprecision(12) << pval;
        //     }
        //     else
        //     {
        //         logger->critical("[Node " + m_node_id + "] Unable to open encoder memory file to write......");
        //     }
        // }
    }

    if (idx == ACTUAL_VELOCITY_FAULHABER_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Actual Velocity
    {
        m_actual_vel = rpdo_mapped[ACTUAL_VELOCITY_FAULHABER_IDX][0]; // Update actual velocity
        m_actual_vel_SI = double(m_actual_vel) / m_conversion_factor; // Update actual velocity in SI units. It differes based on Brand
    }
    if (idx == ACTUAL_TORQUE_FAULHABER_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Torque
    {
        m_current = rpdo_mapped[ACTUAL_TORQUE_FAULHABER_IDX][0]; // Update torque
        m_current_SI = double(m_current) / 1000;                 // Convert to SI unit
    }
    if (idx == ACTUAL_VELOCITY_MAXON_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Actual Velocity
    {
        m_actual_vel = rpdo_mapped[ACTUAL_VELOCITY_MAXON_IDX][0]; // Update actual velocity
        m_actual_vel_SI = double(m_actual_vel) / m_conversion_factor;   // Update actual velocity in SI units. It differes based on Brand
    }
    if (idx == ACTUAL_CURRENT_MAXON_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Current
    {
        m_current = rpdo_mapped[ACTUAL_CURRENT_MAXON_IDX][0]; // Update current or torque
        m_current_SI = double(m_current) / 1000;              // Convert to SI unit
    }

    // you can add more PDOs here to be recognized
    // if Received unknown PDO
    if (!((idx == ACTUAL_TORQUE_FAULHABER_IDX) || (idx == ACTUAL_CURRENT_MAXON_IDX) || (idx == ACTUAL_VELOCITY_FAULHABER_IDX) || (idx == ACTUAL_VELOCITY_MAXON_IDX) || (idx == STATUS_WORD_IDX) || (idx == ACTUAL_POSITION_IDX)))
    {
        std::stringstream ss;
        ss << "[Node " + m_node_id << ": unknown PDO - idx: 0x" << std::hex << idx << " subidx: 0x" << std::hex << subidx; // Convert to hex, uppercase letters
        logger->warn(ss.str());
    }

    if (m_print_pdos)
    {
        std::cout << "<= rPDO1 [" << m_node_id << "]: pos: " << std::dec << m_actual_pos << "  \tStatus: 0b" << ToBinaryString(m_status_word.statusword) << std::endl;
        std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x6064 << "][" << static_cast<int>(0) << "]: " << std::dec << m_actual_pos << std::endl;
        std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x6041 << "][" << static_cast<int>(0) << "]: " << std::hex << "0x" << m_status_word.statusword << std::endl;
    }
    if (m_print_pdos)
    {
        std::cout << "<= rPDO2 [" << m_node_id << "]: Vel: " << std::dec << m_actual_vel << "  \tToruqe: " << m_current << std::endl;
        std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x6077 << "][" << static_cast<int>(0) << "]: " << std::dec << m_current << std::endl;
        std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x606c << "][" << static_cast<int>(0) << "]: " << std::dec << m_actual_vel << std::endl;
    }
}

/* */
void CiA301Node::OnSync(uint8_t cnt, const time_point &t) noexcept
{
    (void) cnt;
    (void) t;
    // Send the scaled_position_command to the slave's object dictionary entry 0x6064.
    try
    {
    }
    catch (const canopen::SdoError &e)
    {
        // Handle SDO write error
        std::cerr << "SDO write error: " << e.what() << std::endl;
    }
}

// ============================== CiA402 State Machine Methods ==============================
/* A member function to switch on the driver. Supports CiA402 State Machine Application Layer.
    The Switch_ON function should be called in the OnBoot callback function.*/
void CiA301Node::SwitchOn()
{
    std::string message;
    m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get statusword on SDO
    message = m_status_word.getCiA402StatusMessage();
    if (m_status_word.Switch_On_Disabled)
    {
        m_control_word.switch_ON = 0;
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 1;
        m_control_word.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get())); // shut down (switch on disable)
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
        message = message + "  =>  " + m_status_word.getCiA402StatusMessage();
    }
    if ((m_status_word.ready_to_switch_ON && !m_status_word.switched_ON) ||
        m_status_word.operation_enabled)
    {
        m_control_word.switch_ON = 1;
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 1;
        m_control_word.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get())); // switch on or disable operation
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
        message = message + "  =>  " + m_status_word.getCiA402StatusMessage();
    }

    if (m_status_word.getCiA402StatusMessage() == "Switched On")
    {
        logger->debug("[Node " + m_node_id + "] " + message);
    }
    else
    {
        logger->error("[Node " + m_node_id + "] " + message);
    }
}

/** A member function to enabled / disable the driver operation
    supports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
void CiA301Node::EnableOperation(const bool enable)
{
    int max_attempts = 10;
    int attempt_count = 0;
    m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO)

    // enable operation
    if (enable && !m_status_word.operation_enabled)
    {
        while (!m_status_word.operation_enabled)
        {
            if (attempt_count >= max_attempts) // If the loop has run 10 times without enabling the operation, return an error
            {
                logger->error("[Node " + m_node_id + "] Operation not enabled after " + std::to_string(max_attempts) + " attempts - current status: " + m_status_word.getCiA402StatusMessage());
                throw std::runtime_error("Error: Operation not enabled after 10 attempts");
            }
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x000F));                  // set the state macine to enabled operation
            Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                 // wait for safety
            m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
            attempt_count++;
        }
        logger->debug("[Node " + m_node_id + "] " + m_status_word.getCiA402StatusMessage());
    }
    // switch on or disable operation
    else if (!enable && m_status_word.operation_enabled)
    {
        while (m_status_word.operation_enabled)
        {
            if (attempt_count >= max_attempts) // If the loop has run 10 times without enabling the operation, return an error
            {
                logger->critical("[Node " + m_node_id + "] Operation not disabled after " + std::to_string(max_attempts) + " attempts - current status: " + m_status_word.getCiA402StatusMessage());
            }
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x0007));                  // set the state macine to switched on
            Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                 // wait for safety
            m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
            attempt_count++;
        }
        logger->debug("[Node " + m_node_id + "] " + m_status_word.getCiA402StatusMessage());
    }
    else if (enable && m_status_word.operation_enabled)
    {
        logger->warn("[Node " + m_node_id + "] request to enable - alraedy enbaled");
    }
    else
    {
        logger->warn("[Node " + m_node_id + "] request to disable - alraedy disabled");
    }
}

/** A member function to enabled / disable the driver operation
    supports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
// ********************** this function is under development **********************
void CiA301Node::EnableOperationWithPdo(const bool enable)
{
    (void) enable;
    // using TPDO1
    // m_control_word.enable_operation = 1;
    // tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // enable
    // tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();
    // flag_operation_enabled = true;

    //     m_control_word.enable_operation = 0;
    // tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // disbale
    // tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();
    // flag_operation_enabled = false;
}

/* A member funtion to reset faults using SDO
    should be called in oOnConfig*/
void CiA301Node::ResetFault()
{
    m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // update status word
    if (m_status_word.fault)
    {
        // logger->warn("[Node " + node_id + "] fault code" + std::to_string(Wait(AsyncRead<uint16_t>(0x2321, 0x0000))), true); // update status word

        m_control_word.fault_reset = 0;                                           // make the bit zero to rise it later for reserting fault
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));    // set the controlword on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                 // wait for safety
        m_control_word.fault_reset = 1;                                           // fault resets at rising edge
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));    // set the controlword on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                 // wait for safety
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
        logger->warn("[Node " + m_node_id + "] Reset fault => Fault status: " + std::to_string(m_status_word.fault));
    }
    else
    {
        logger->debug("[Node " + m_node_id + "] No fault to reset");
    }
}

/*  This task is reponsible for cyclic sending target positions or target velocities through PDO2 / PDO3
    This task is posted in in the OnBoot function */
void CiA301Node::TaskTarget() noexcept
{
    while (true)
    {
        if (robot_states->m_flag_operation_enabled & !m_isConfiguring)
        {
            if (m_current_operation_mode == OpMode::PositionProfile) // Position Profile mode
            {
                m_control_word.bit4 = 1; // set new target bit (bit 4)
                m_control_word.bit5 = 1; // set immediately bit (bit 5)
                if (m_flags.get(Flags::FlagIndex::NEW_TARG_READY))
                {
                    if (m_target_pos != m_target_pos_prev)
                    {
                        tpdo_mapped[TARGET_POSITION_IDX][0] = m_target_pos; // target position
                        tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get();
                        tpdo_mapped[TARGET_POSITION_IDX][0].WriteEvent(); // Trigger write events for PDO1.
                        m_target_pos_prev = m_target_pos;
                        m_flags.set(Flags::FlagIndex::NEW_TARG_READY, false);
                        m_status_word.bit10 = 0;

                        if (m_print_pdos)
                        {
                            std::cout << "=> tPDO2 [" << m_node_id << "]: target: " << std::dec << m_target_pos << std::endl;
                            std::cout << "         [" << m_node_id << "]:[" << std::hex << CONTROL_WORD_IDX << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(m_control_word.get()) << std::hex << "   (0x" << m_control_word.get() << ")" << std::endl;
                        }
                    }
                }
                else
                {
                    logger->error("[Node " + m_node_id + "] Motion controller is not ready to receive a new position target! (PDO is sending too fast - please wait!)");
                }
            }
            // velocity profile mode is not verified - may need some improvement
            else if (m_current_operation_mode == OpMode::VelocityProfile) // Velocity Profile mode
            {
                m_control_word.enable_operation = 1;                     // set immediately bit (bit 5)
                tpdo_mapped[TARGET_VELOCITY_IDX][0] = m_target_vel;      // target velocity
                tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // enable
                tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();        // Trigger write events for PDO1.
                                                                         // std::cout << "target vel: " << vel << std::endl;
                                                                         // if (flag_printlog)
                                                                         // {
                                                                         //     std::cout << "=> tPDO2 [" << node_id << "]: target: " << std::dec << vel << std::endl;
                                                                         //     std::cout << "         [" << node_id << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(ctrl) << std::hex << "   (0x" << ctrl << ")" << std::endl;
                                                                         // }
            }
            // else if (m_current_operation_mode == OpMode::Velocity) // Velocity mode
            // {
            //     // shared_state->Log_message("[Node " + node_id +
            //     //                           ": sending velocity mode setting = " + std::to_string(m_target_vel),
            //     //                       true);
            //     m_control_word.enable_operation = 1;                     // set immediately bit (bit 5)
            //     tpdo_mapped[0x206B][0] = m_target_vel;                   // target velocity
            //     tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // enable
            //     tpdo_mapped[0x206B][0].WriteEvent();                     // Trigger write events for PDO1.
            //                                                              // std::cout << "target vel: " << vel << std::endl;
            //                                                              // if (flag_printlog)
            //                                                              // {
            //                                                              //     std::cout << "=> tPDO2 [" << node_id << "]: target: " << std::dec << vel << std::endl;
            //                                                              //     std::cout << "         [" << node_id << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(ctrl) << std::hex << "   (0x" << ctrl << ")" << std::endl;
            //                                                              // }
            // }
        }
        else
        {
            logger->debug("[Node " + m_node_id + "] Robot is not operational");
        }

        Wait(AsyncWait(duration(std::chrono::milliseconds(m_sample_time))));
    }
}

/*  This task is reponsible for changing ccontroller configuration (currently setting zero positon).
    Other tasks should be halted using flag_config when this task is actioning*/
void CiA301Node::TaskConfig() noexcept
{
    while (true)
    {
        if (m_isConfiguring)
        {
            CiA301Node::SetEncoder(0.0);
            CiA301Node::EnableOperation(true);
            Wait(AsyncWait(duration(std::chrono::microseconds(m_sample_time))));
            m_isConfiguring = false;
        }
        Wait(AsyncWait(duration(std::chrono::microseconds(m_sample_time))));
    }
}

// ============================== Configuration Methods ==============================
/** A member function to set the mode of operation. The state machine needs fo be in switch on (disabled operation) before using
    @param operationMode:   An integer defining the mode of operation based on the motion controller datasheet
                            0=disable, 1=PP, 3=VP, 6=Home, -1=FaulhaberCommand*/
void CiA301Node::SetOperationMode(OpMode operationMode)
{
    logger->debug("[Node " + m_node_id + "] Setting mode of operation to " + std::to_string(int8_t(operationMode)));
    m_current_operation_mode = operationMode;

    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));

    Wait(AsyncWrite<int8_t>(MODE_OF_OPERATION_IDX, 0, static_cast<int8_t>(m_current_operation_mode))); // Set the mode of operation
    int8_t op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));                  // read the actual model of operation
    while (static_cast<int8_t>(op_mod_disp) != static_cast<int8_t>(m_current_operation_mode))          // wait until the mode change is confirmed
    {
        op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
    }
    logger->debug("[Node " + m_node_id + "] Mode of operation = " + std::to_string(int(op_mod_disp)));
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order
}

/** This function sets the transmission type of a specific Transmit Process Data Object (TPDO).
    @param numPDO: An integer specifying the PDO number (1-4) to configure.
    @param transmisionType: An integer representing the desired transmission type
        (see CANopen SYNC object documentation) for the PDO.
        0 => synchronous, acyclic   (after SYNC if it has been changed)
        1 => synchronous, cyclical  (after SYNC)
        255 => asynchronous (event controlled)
        ... */
void CiA301Node::SetTpdoTranstype(const int NumPDO, const int TransmisionType)
{
    if (TransmisionType >= 0 && TransmisionType <= 255 && TransmisionType != 254)
    {
        switch (NumPDO)
        {
        case 1:
            Wait(AsyncWrite<int8_t>(0x1800, 2, TransmisionType)); // Configure TPDO1 transmission type.
            logger->debug("[Node " + m_node_id + "] Setting TPDO1 transmission type to " + std::to_string(TransmisionType));
            break;
        case 2:
            Wait(AsyncWrite<int8_t>(0x1801, 2, TransmisionType)); // Configure TPDO2 transmission type.
            logger->debug("[Node " + m_node_id + "] Setting TPDO2 transmission type to " + std::to_string(TransmisionType));
            break;
        case 3:
            Wait(AsyncWrite<int8_t>(0x1802, 2, TransmisionType)); // Configure TPDO3 transmission type.
            logger->debug("[Node " + m_node_id + "] Setting TPDO3 transmission type to " + std::to_string(TransmisionType));
            break;
        case 4:
            Wait(AsyncWrite<int8_t>(0x1803, 2, TransmisionType)); // Configure TPDO4 transmission type.
            logger->debug("[Node " + m_node_id + "] Setting TPDO4 transmission type to " + std::to_string(TransmisionType));
            break;
        default:
            logger->warn("[Node " + m_node_id + "] Invalid TPDO number. TPDO 1-4 are supported");
            break;
        }
    }
    else
    {
        logger->warn("[Node " + m_node_id + "] Invalid transmission type. Type must be in the range [0, 255] excluding 254.");
    }
}

/** Member function to set the position profile parameters for the node.
   @param MaxAcc: Maximum acceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxDcc: Maximum deceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxVel: Maximum velocity     for CiA301 => [pulse/s]      for FC => [mm/s]     */
void CiA301Node::SetProfileParams(const int MaxAcc, const int MaxDcc, const int MaxVel)
{
    if (m_controller_brand == "Maxon")
    {
        logger->debug("[Node " + m_node_id +
                      ": Setting profile position parameters to: " +
                      "ACC: " + std::to_string(MaxAcc) + " [rpm/s]  |  " +
                      "DCC: " + std::to_string(MaxDcc) + " [rpm/s]  |  " +
                      "Vel: " + std::to_string(MaxVel) + " [rpm]  ");
        Wait(AsyncWrite<uint32_t>(0x6065, 00, static_cast<uint32_t>(1000000)));   // Max. Following Error
        Wait(AsyncWrite<int32_t>(0x607D, 01, static_cast<int32_t>(-2147483648))); // Min. Position Limit    (PP)
        Wait(AsyncWrite<int32_t>(0x607D, 02, static_cast<int32_t>(2147483647)));  // Max. Position Limit    (PP)
        Wait(AsyncWrite<uint32_t>(0x6081, 00, static_cast<uint32_t>(MaxVel)));    // set profile velocity   (PP)
        Wait(AsyncWrite<uint32_t>(0x6083, 00, static_cast<uint32_t>(MaxAcc)));    // set profile acc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x6084, 00, static_cast<uint32_t>(MaxDcc)));    // set profile dcc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x60C5, 00, static_cast<uint32_t>(10000000)));  // set max ACC        (Velocity mode)

        // Wait(AsyncWrite<int32_t>(0x6086, 00, static_cast<uint16_t>(0))); // Max. Position Limit
    }
    else
    {
        logger->debug("[Node " + m_node_id +
                      ": Setting profile position parameters to: " +
                      "ACC: " + std::to_string(MaxAcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "DCC: " + std::to_string(MaxDcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "Vel: " + std::to_string(MaxVel) + " [0.01mm/s] or [0.1deg/s]  ");
        // Wait(AsyncWrite<uint32_t>(0x6065, 00, static_cast<uint32_t>(1000000)));   // Max. Following Error
        // Wait(AsyncWrite<int32_t>(0x607D, 01, static_cast<int32_t>(-2147483648))); // Min. Position Limit    (PP)
        // Wait(AsyncWrite<int32_t>(0x607D, 02, static_cast<int32_t>(2147483647)));  // Max. Position Limit    (PP)
        Wait(AsyncWrite<uint32_t>(0x6081, 00, static_cast<uint32_t>(MaxVel))); // set profile velocity   (PP)
        Wait(AsyncWrite<uint32_t>(0x6083, 00, static_cast<uint32_t>(MaxAcc))); // set profile acc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x6084, 00, static_cast<uint32_t>(MaxDcc))); // set profile dcc    (PP & PV)

        // Wait(AsyncWrite<uint32_t>(0x60C5, 00, static_cast<uint32_t>(10000000)));  // set max ACC        (Velocity mode)
        // Wait(AsyncWrite<int32_t>(0x6086, 00, static_cast<uint16_t>(0))); // Max. Position Limit
    }
}

/** This function sets the current position of the encoder. It is verified only for CiA301
    Other tasks should be on hold when "Set_encoder" is running
    @param offset set value for the current positon in userdefined unit (see Faulhaber documentation)*/
void CiA301Node::SetEncoder(const double value)
{
    bool flag_home_done = false;
    logger->debug("[Node " + m_node_id + "] Setting encoder value...");

    CiA301Node::EnableOperation(false);                                              // disable and wait to make sure motion is stopped - disabling is necessary to change the mode of operation
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                       // wait for better log order
    CiA301Node::SetOperationMode(OpMode::Homing);                                    // switch to Homing mode
    int8_t homing_method = (m_controller_brand == "Maxon") ? 35 : 37;                // Homing method 35 for Maxon and 37 for Faulhaber
    Wait(AsyncWrite<int8_t>(0x6098, 00, static_cast<int8_t>(homing_method)));        // set Homing method
    CiA301Node::EnableOperation(true);                                               // must be enabled before start homing
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                       // wait for better log order
    Wait(AsyncWrite<int32_t>(0x607C, 00, static_cast<int32_t>(-1 * value * m_ppu))); // set Homing offset
    m_control_word.enable_operation = 1;                                             // must be enabled before start homing
    m_control_word.bit4 = 0;                                                         // set Bit 4 to 0 to tuggle to 1 later for start of homing
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));           // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                        // wait for safety
    m_control_word.bit4 = 1;                                                         // set Bit 4 0->1 (rising edge means start homing)
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));           // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                        // wait for safety
    logger->debug("[Node " + m_node_id + "] encoder set value: " + std::to_string(value) + " | pulse count: " + std::to_string(static_cast<int32_t>(-1 * value * m_ppu)));
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    while (!flag_home_done) // wait until homing is confirmed
    {
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                 // wait for safety
        logger->debug("[Node " + m_node_id + "] enable: " + std::to_string(m_status_word.operation_enabled) + ": bit10: " + std::to_string(m_status_word.bit10) + ": bit12: " + std::to_string(m_status_word.bit12) + ": bit13: " + std::to_string(m_status_word.bit13));
        if (m_status_word.bit12 && m_status_word.bit10) // check the bit (12-homing attained) and bit (10-target reached)
        {
            m_control_word.bit4 = 0;                                               // Bit 4 to 0
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get())); // send controlword on SDO
            flag_home_done = true;                                                 // set homing finished flag
            logger->debug("[Node " + m_node_id + "] Current position set to " + std::to_string(value));
            m_flags.set(Flags::FlagIndex::ENCODER_SET, true);
        }
        if (m_status_word.bit13)
        {
            // m_encoder_memory_file.close();
            logger->critical("[Node " + m_node_id + "] A homing error has occurred");
        }
    }
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    CiA301Node::EnableOperation(false);                        // disable before changing operation mode
    CiA301Node::SetOperationMode(m_operation_mode);            // bring back to original operation mode
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    // Node::Set_operation_mode(0);
    // Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
}

/* Sets in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void CiA301Node::SetHomeOffsetValue(const double val)
{
    m_pos_offset_SI = val;
}

/*needs to be verified and tested*/
void CiA301Node::FindFowrardLimits()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    CiA301Node::GetActualPos(pos_0);

    CiA301Node::EnableOperation(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    CiA301Node::SetOperationMode(OpMode::VelocityProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    CiA301Node::EnableOperation(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_control_word.enable_operation = 1;                                                 // set immediately bit (bit 5)
    tpdo_mapped[TARGET_VELOCITY_IDX][0] = static_cast<int32_t>(m_vel_findlimit * m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get();                             // enable
    tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();                                    // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        CiA301Node::GetCurrent(current);

        currentBuffers.push_back(current);
        if (currentBuffers.size() > maxBufferSize)
        {
            currentBuffers.pop_front(); // Keep the buffer size at 100
        }
        double sum = 0.0;
        for (auto it = currentBuffers.begin(); it != currentBuffers.end(); ++it)
        {
            sum += *it;
        }
        current_avg = sum / static_cast<double>(currentBuffers.size());

        if (abs(current) >= m_current_threshold && !limit_reached && currentBuffers.size() == maxBufferSize)
        {
            limit_reached = true;
            CiA301Node::EnableOperation(false);
            CiA301Node::GetActualPos(pos_1);
            m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_node_id << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    CiA301Node::SetTargetVel(0.0);
    CiA301Node::EnableOperation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

// ============================== Command Methods ==============================
/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void CiA301Node::SetTargetPosAbs(const double val)
{
    m_target_pos_SI = val;
    m_target_pos = static_cast<int32_t>(m_target_pos_SI * m_ppu);
}

/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void CiA301Node::SetTargetPos(const double val)
{
    m_target_pos_SI = val + m_pos_offset_SI;
    m_target_pos = static_cast<int32_t>(m_target_pos_SI * m_ppu);
}

/* Sets target valocit in [m/s] or [rad/s] and converts motion controller unit
    and updates the associates variable to be executes in the next sample*/
void CiA301Node::SetTargetVel(const double val)
{
    m_target_vel_SI = val;
    m_target_vel = static_cast<int32_t>(m_target_vel_SI * m_conversion_factor);
}

/*needs to be verified and tested*/
void CiA301Node::BackToZero()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    CiA301Node::GetActualPos(pos_0);

    CiA301Node::EnableOperation(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    CiA301Node::SetOperationMode(OpMode::PositionProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    CiA301Node::EnableOperation(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_control_word.enable_operation = 1;                                                 // set immediately bit (bit 5)
    tpdo_mapped[TARGET_VELOCITY_IDX][0] = static_cast<int32_t>(m_vel_findlimit * m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get();                             // enable
    tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();                                    // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        CiA301Node::GetCurrent(current);

        currentBuffers.push_back(current);
        if (currentBuffers.size() > maxBufferSize)
        {
            currentBuffers.pop_front(); // Keep the buffer size at 100
        }
        double sum = 0.0;
        for (auto it = currentBuffers.begin(); it != currentBuffers.end(); ++it)
        {
            sum += *it;
        }
        current_avg = sum / static_cast<double>(currentBuffers.size());

        if (abs(current) >= m_current_threshold && !limit_reached && currentBuffers.size() == maxBufferSize)
        {
            limit_reached = true;
            CiA301Node::EnableOperation(false);
            CiA301Node::GetActualPos(pos_1);
            m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_node_id << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    CiA301Node::SetTargetVel(0.0);
    CiA301Node::EnableOperation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

// ============================== Feedback Methods ==============================
/* accessor to motor current [A]*/
void CiA301Node::GetCurrent(double &currentPtr) const
{
    currentPtr = m_current_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void CiA301Node::GetActualPosAbs(double &actualPosPtr) const
{
    actualPosPtr = m_actual_pos_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void CiA301Node::GetActualPos(double &actualPosPtr) const
{
    actualPosPtr = m_actual_pos_SI - m_pos_offset_SI;
}

/* accessor to actual motor velocity [m/s] or [rad/s]*/
void CiA301Node::GetActualVel(double &actualVelPtr) const
{
    // *actualVelPtr = double(vel_actual / (60.0 / gear_ratio));
    actualVelPtr = m_actual_vel_SI;
}

/* if target is reached */
bool CiA301Node::IsReached() const
{
    if (m_current_operation_mode == OpMode::PositionProfile || m_current_operation_mode == OpMode::VelocityProfile)
    {
        return m_status_word.bit10;
    }
    else
    {
        logger->debug("[Node " + m_node_id + "]Target reached is only defined for PP and PV modes ");
        return 0;
    }
}

// ============================== Utility Methods ==============================
/* need a cleanup*/
void CiA301Node::ReadEncoderMemFile(const std::string &directory, double &encoder_memory, std::ofstream &encoder_memory_file)
{
    // check for encoder memory file to set
    std::stringstream file_name;
    file_name << directory << "encoder_memory_node_" << m_node_id << ".dat";

    if (std::filesystem::exists(file_name.str())) // Check if the file exists
    {
        // std::cout << "[Node " << id << ": Encoder memory file axists" << std::endl;
        std::ifstream temp(file_name.str());
        if (temp.is_open())
        {
            try
            {
                std::string content;
                temp >> content;                     // Read the content as a string
                encoder_memory = std::stod(content); // Convert the string to double
                logger->debug("[Node " + m_node_id + "] Encoder memory = " + std::to_string(encoder_memory));
                temp.close();
            }
            catch (const std::exception &e)
            {
                // Handle the exception here, e.g., log an error message
                temp.close();
                logger->critical("[Node " + m_node_id + "] Error reading encoder memory. Check memory files: " + e.what());
            }
        }
        else
        {
            logger->critical("[Node " + m_node_id + "] Unable to open encoder memory file to read");
        }

        (encoder_memory_file).open(file_name.str(), std::ios::out);
        if (!(encoder_memory_file).is_open())
        {
            logger->critical("[Node " + m_node_id + "] Unable to open encoder memory file to write");
        }
    }
    else
    {
        encoder_memory = 0;
        (encoder_memory_file).open(file_name.str(), std::ios::out);
        if ((encoder_memory_file).is_open())
        {
            encoder_memory_file << 0;
            logger->debug("[Node " + m_node_id + "] Memory file built and opened");
        }
        else
        {
            logger->critical("[Node " + m_node_id + "] Unable to open built encoder memory file to write");
        }
    }
}

// Getter function to retrieve the StatusWord
StatusWord CiA301Node::GetStatusword() const
{
    return m_status_word;
}

// Getters
bool CiA301Node::GetNodeFlags(Flags::FlagIndex index) const
{
    return m_flags.get(index);
}