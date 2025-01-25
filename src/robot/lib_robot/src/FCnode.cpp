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
  This class is responsible for creating nodes that facilitate communication with Faulhaber motion controller
  over the CANopen protocol. It supports  Faulhaber channel commands (which is now considered obsolete), and
  CiA402 device profile .
*/

#include "FCnode.hpp"

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
FCNode::FCNode(ev_exec_t * /*exec*/,
               canopen::AsyncMaster &master,
               unsigned int NodeID,
               std::string ControllerBrand,
               double EncoderResolution,
               double GearRatio,
               double VelocityFactor,
               unsigned int SampleTime,
               OpMode OperationMode,
               double ProfileAcc,
               double ProfileVel,
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
    this->m_velfactor = VelocityFactor;
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
        double conversion_factor = (m_controller_brand == "Maxon") ? (60.0 / m_gear_ratio) : (m_ppu / VelocityFactor);
        m_profile_vel_SI = static_cast<unsigned int>(abs(ProfileVel * conversion_factor));
        m_profile_acc_SI = static_cast<unsigned int>(abs(ProfileAcc * conversion_factor));
    }
    else
    {
        logger->critical("[Node " + m_node_id + "] Unknown motion controller brand | Moxan, and Faulhaber are supported");
    }

    logger->debug("[Node " + m_node_id + "] " +
                  "Controller brand: " + this->m_controller_brand + " - " +
                  "Operation layer: " + " Faulhaber Command - " +
                  "PPU: " + std::to_string(this->m_ppu) + " - " +
                  "Gear ratio: " + std::to_string(GearRatio) + " - " +
                  "Encoder resoluton: " + std::to_string(EncoderResolution));
}

FCNode::~FCNode()
{
    m_encoder_memory_file.close();
}

/* This function gets called during the boot-up process for the node. The
    'res' parameter is the function that MUST be invoked when the configuration
    is complete. Because this function runs as a task inside a coroutine, it
    can suspend itself and wait for an asynchronous function, such as an SDO
    request, to complete. */
void FCNode::OnConfig(std::function<void(std::error_code ec)> res) noexcept
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
void FCNode::OnBoot(canopen::NmtState /*st*/, char es, const std::string &what) noexcept
{
    if (!es || es == 'L')
    {
        // Successful boot-up or boot-up is in progress ("L" state).
        m_flags.set(Flags::FlagIndex::BOOT_SUCCESS, true);
        logger->debug("[Node " + m_node_id + "] Booted successfully.");

        while (!robot_states->m_boot_success)                         // wait for all other nodes to boot
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for all other nodes to boot

        FCNode::ResetFault();                                         // reset faults if there is any
        FCNode::SwitchOn();                                           // switch ON
        while (!robot_states->m_flag_robot_switched_on)               // wait for other nodes to switch ON
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to switch ON
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));    // wait for better log

        logger->debug("[Node " + m_node_id + "] Setting mode of operation to " + std::to_string(int8_t(OpMode::FaulhaberCommand)));
        Wait(AsyncWrite<int8_t>(MODE_OF_OPERATION_IDX, 0, static_cast<int8_t>(OpMode::FaulhaberCommand))); // Set the mode of operation
        int8_t op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));                  // read the actual model of operation
        while (static_cast<int8_t>(op_mod_disp) != static_cast<int8_t>(OpMode::FaulhaberCommand))          // wait until the mode change is confirmed
        {
            op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));
            Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        }
        logger->debug("[Node " + m_node_id + "] Mode of operation = " + std::to_string(int(op_mod_disp)));
        Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

        // FCNode::SetOperationMode(m_operation_mode);
        // set mode of operation
        FCNode::SetProfileParams(this->m_profile_acc_SI, this->m_profile_acc_SI, this->m_profile_vel_SI); // set profile parameters
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));                                        // wait for for a clean log file

        // ### This part is specific to FC application layer no CiA301 ###//
        FCNode::FaulhaberTraceConfig();                           // set Trace Config for tPDO3
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        FCNode::SetTpdoTranstype(1, 0);                           // set tPDO0 transmission type to be asynchrnous
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        FCNode::SetTpdoTranstype(2, 1);                           // set tPDO2 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        FCNode::SetTpdoTranstype(3, 1);                           // set tPDO3 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file

        FCNode::EnableOperation(true);                                // enable operation before posting the tasks
        while (!robot_states->m_flag_operation_enabled)               // wait for other nodes to enabled
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to enabled
        Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));   // wait for a clean log

        Post(&FCNode::TaskTargetFC, this);
        m_flags.set(Flags::FlagIndex::TASKS_POSTED, true);
        logger->debug("[Node " + m_node_id + "] Task posted");
    }
    else
    {
        logger->error("[Node " + m_node_id + "] Failed to boot: " + what);
    }
}

/* This function is similar to OnConfg(), but it gets called by the
    AsyncDeconfig() method of the master. */
void FCNode::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept
{
    logger->debug("[Node " + m_node_id + "] Deconfiging");
    try
    {
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 0;
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get())); // Quick Stop
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word
        logger->debug("[Node " + m_node_id + "] " + m_status_word.getCiA402StatusMessage());
        m_encoder_memory_file.close();
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
void FCNode::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
    // in Faulhaber Command, Transsmit PDO1 is set for statusword
    // Transsmit PDO2 responds to the request of Receved PDO2, i.e. if you want to recieve current postition, you should
    // send a PDO2 with comment POS (or anything else) and value 0 to get the response of position. I do it at OnSync. So,
    // once the Transsmit PDO2 is received by the master, the command should be checked to see what the message is.
    // Transsmit PDO3, could be set to send up to two predefined value using TraceConfig (See page 72 of Faulhave Communication Manual).
    // The number of bytes of the data received should be set according to the size of tha package defined in .eds file
    // under [2304sub1] and [2304sub2].

    if (idx == STATUS_WORD_IDX && subidx == 0) // rPDO1 (PDO1 from slave) - Status word
    {
        m_status_word.update(rpdo_mapped[STATUS_WORD_IDX][0]);
        if (m_print_pdos)
        {
            std::cout << "<= rPDO1 [" << m_node_id << "]: Status: 0b" << ToBinaryString(m_status_word.statusword) << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x6041 << "][" << static_cast<int>(0) << "]: " << std::hex << "0x" << m_status_word.statusword << std::endl;
        }
    }
    else if (idx == 0x2302 && subidx == 1) // rPDO2 (PDO2 from slave) - Faulhaber Command
    {
        uint8_t comcode = rpdo_mapped[0x2301][1];
        int32_t value = rpdo_mapped[0x2302][1];
        std::string command = GetCommandFromHex(comcode);
        if (m_print_pdos)
        {
            std::cout << "<= rPDO2 [" << m_node_id << "]: Command: " << command << "  \tvalue: " << std::dec << value << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2301 << "][" << static_cast<int>(1) << "]: "
                      << "0x" << std::hex << static_cast<int>(comcode) << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2302 << "][" << static_cast<int>(1) << "]: " << std::dec << value << std::endl;
        }

        if (command == "POS")
        {
            // m_actual_pos_SI = double(value) / double(m_ppu);
        }
    }
    else if (idx == 0x2304 && subidx == 2) // rPDO3 (PDO3 from slave) - Extended command
    {
        int32_t value_1 = rpdo_mapped[0x2304][1]; // set to position according to traceconfig
        int16_t value_2 = rpdo_mapped[0x2304][2]; // set to velocity according to traceconfig
        if (m_print_pdos)
        {
            std::cout << "<= rPDO3 [" << m_node_id << "]: actual vel: " << std::dec << value_1 << "  \tcurrent: " << std::dec << value_2 << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2304 << "][" << static_cast<int>(1) << "]: " << std::dec << value_1 << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2304 << "][" << static_cast<int>(2) << "]: " << std::dec << value_2 << std::endl;
        }

        m_actual_pos_SI = double(value_1) / double(m_ppu);
        m_actual_vel_SI = double(value_2) / (60 / m_gear_ratio / m_velfactor); // convert velocity to user defined unit
        // m_current_SI = double(m_current) / 1000;                // convert to SI unit
    }
}

/* */
void FCNode::OnSync(uint8_t cnt, const time_point &t) noexcept
{
    (void) cnt;
    (void) t;
    try
    {
        // is part is for if you want to receive information on PDO2, this part request the parameter that you need
        // Node::FaulhaberCommand("POS", 0);
        // Wait(AsyncWait(duration(std::chrono::microseconds(20))));
    }
    catch (const canopen::SdoError &e)
    {
        // Handle SDO write error
        std::cerr << "SDO write error: " << e.what() << std::endl;
    }
}

/*  This task is reponsible for */
void FCNode::TaskTargetFC() noexcept
{
    while (true)
    {
        if (m_operation_mode == OpMode::PositionProfile)
        {
            // logger->critical("[Node " + m_node_id + "] Checkpoint " + std::to_string(m_target_pos));
            FCNode::FaulhaberCommand("LA", m_target_pos);
            Wait(AsyncWait(duration(std::chrono::microseconds(10))));
            FCNode::FaulhaberCommand("M", 0x0000);
            Wait(AsyncWait(duration(std::chrono::milliseconds(m_sample_time))));
        }
        else if (m_operation_mode == OpMode::VelocityProfile)
        {
            FCNode::FaulhaberCommand("V", m_target_vel);
            // Wait(AsyncWait(duration(std::chrono::microseconds(10))));
            // FCNode::FaulhaberCommand("M", 0x0000);
            Wait(AsyncWait(duration(std::chrono::milliseconds(m_sample_time))));
        }
    }
}

/* A member function to switch on the driver. Supports CiA402 State Machine Application Layer.
    The Switch_ON function should be called in the OnBoot callback function.*/
void FCNode::SwitchOn()
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
        throw std::runtime_error("Error: Not switched ON");
    }
}

/** A member function to enabled / disable the driver operation
    upports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
void FCNode::EnableOperation(bool enable)
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
    upports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
// ********************** this function is under construction **********************
void FCNode::EnableOperationWithPdo(const bool enable)
{
    (void) enable;
    // Node::FaulhaberCommand("EN", 0); // enable
    // Wait(AsyncWait(duration(std::chrono::microseconds(10))));
    // flag_operation_enabled = true;

    // Node::FaulhaberCommand("DI", 0); // enable
    // Wait(AsyncWait(duration(std::chrono::microseconds(10))));
    // flag_operation_enabled = true;

    // using SDO (SDO method cannot be called out of the Task loop. It causes segmentation fault error)
    // Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x000F));
    // uint16_t statusword = Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000));
    // while (getCiA402StatusMessage(statusword) != "Operation enabled")
    // {
    //     statusword = Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000));
    // }
    // statusword = Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000));
    // std::cout << "[Node " << node_id << ": " << getCiA402StatusMessage(statusword) << std::endl;
    // if (getCiA402StatusMessage(statusword) == "Operation enabled")
    // {
    //     flag_operation_enabled = true; = true;
    // }
}

/* A member funtion to reset faults using SDO
    should be called in oOnConfig*/
void FCNode::ResetFault()
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

/** A member function to set the mode of operation. The state machine needs fo be in switch on (disabled operation) before using
    @param operationMode:   An integer defining the mode of operation based on the motion controller datasheet
                            0=disable, 1=PP, 3=VP, 6=Home, -1=FaulhaberCommand*/
void FCNode::SetOperationMode(const OpMode operationMode)
{
    logger->debug("[Node " + m_node_id + "] Setting mode of operation to " + std::to_string(int8_t(operationMode)));
    m_current_operation_mode = operationMode;

    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));

    Wait(AsyncWrite<int8_t>(MODE_OF_OPERATION_IDX, 0, static_cast<int8_t>(OpMode::FaulhaberCommand))); // Set the mode of operation
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
void FCNode::SetTpdoTranstype(const int NumPDO, const int TransmisionType)
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
            logger->debug("[Node " + m_node_id + "] Invalid TPDO number. TPDO 1-4 are supported");
            break;
        }
    }
    else
    {
        logger->debug("[Node " + m_node_id + "] Invalid transmission type. Type must be in the range [0, 255] excluding 254.");
    }
}

/** Member function to set the position profile parameters for the node.
   @param MaxAcc: Maximum acceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxDcc: Maximum deceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxVel: Maximum velocity     for CiA301 => [pulse/s]      for FC => [mm/s]     */
void FCNode::SetProfileParams(int MaxAcc, int MaxDcc, int MaxVel)
{
    if (MaxAcc < 0)
    {
        MaxAcc = 0; // If maxACC is negative, set it to 0.
        logger->debug("[Node " + m_node_id + "] maxACC was limited to 0.");
    }
    else if (MaxAcc > 30000)
    {
        MaxAcc = 30000; // If maxACC is greater than 30000, set it to 30000.
        logger->debug("[Node " + m_node_id + "] maxACC was limited to 30000.");
    }

    if (MaxDcc < 0)
    {
        MaxDcc = 0; // If maxDCC is negative, set it to 0.
        logger->debug("[Node " + m_node_id + "] maxDCC was limited to 0.");
    }
    else if (MaxDcc > 30000)
    {
        MaxDcc = 30000; // If maxDCC is greater than 30000, set it to 30000.
        logger->debug("[Node " + m_node_id + "] maxDCC was limited to 30000.");
    }

    if (MaxVel < 0)
    {
        MaxVel = 0; // If maxSpeed is negative, set it to 0.
        logger->debug("[Node " + m_node_id + "] maxSpeed was limited to 0.");
    }
    else if (MaxVel > 10000)
    {
        MaxVel = 10000; // If maxSpeed is greater than 30000, set it to 30000.
        logger->debug("[Node " + m_node_id + "] maxSpeed was limited to 30000.");
    }
    {
        logger->debug("[Node " + m_node_id +
                      "] Setting profile position parameters to: " +
                      "ACC: " + std::to_string(MaxAcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "DCC: " + std::to_string(MaxDcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "Vel: " + std::to_string(MaxVel) + " [0.01mm/s] or [0.1deg/s]  ");

        FCNode::FaulhaberCommand("AC", MaxAcc);  // set profile acc
        FCNode::FaulhaberCommand("DEC", MaxDcc); // set profile dcc
        FCNode::FaulhaberCommand("SP", MaxVel);  // set profile velocity
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
    }
}

/** This function sets the current position of the encoder. It is verified only for CiA301
    Other tasks should be on hold when "Set_encoder" is running
    @param offset set value for the current positon in userdefined unit (see Faulhaber documentation)*/
/*######## This function is nor validated for Faulhaber command #######*/
void FCNode::SetEncoder(const double value)
{
    bool flag_home_done = false;
    logger->debug("[Node " + m_node_id + "] Setting encoder value...");

    FCNode::EnableOperation(false);                                                        // disable and wait to make sure motion is stopped - disabling is necessary to change the mode of operation
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                             // wait for better log order
    FCNode::SetOperationMode(OpMode::Homing);                                              // switch to Homing mode
    int8_t homing_method = (this->m_controller_brand == "Maxon") ? 35 : 37;                // Homing method 35 for Maxon and 37 for Faulhaber
    Wait(AsyncWrite<int8_t>(0x6098, 00, static_cast<int8_t>(homing_method)));              // set Homing method
    FCNode::EnableOperation(true);                                                         // must be enabled before start homing
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                             // wait for better log order
    Wait(AsyncWrite<int32_t>(0x607C, 00, static_cast<int32_t>(-1 * value * this->m_ppu))); // set Homing offset
    m_control_word.enable_operation = 1;                                                   // must be enabled before start homing
    m_control_word.bit4 = 0;                                                               // set Bit 4 to 0 to tuggle to 1 later for start of homing
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));                 // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                              // wait for safety
    m_control_word.bit4 = 1;                                                               // set Bit 4 0->1 (rising edge means start homing)
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_control_word.get()));                 // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                              // wait for safety
    logger->debug("[Node " + m_node_id + "] encoder set value: " + std::to_string(value) + " | pulse count: " + std::to_string(static_cast<int32_t>(-1 * value * this->m_ppu)));
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
            m_encoder_memory_file.close();
            logger->critical("[Node " + m_node_id + "] A homing error has occurred");
        }
    }
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    FCNode::EnableOperation(false);                            // disable before changing operation mode
    FCNode::SetOperationMode(m_operation_mode);                // bring back to original operation mode
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    // Node::Set_operation_mode(0);
    // Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
}

/*######## This function is nor validated for Faulhaber command #######*/
void FCNode::FindFowrardLimits()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    FCNode::GetActualPos(pos_0);

    FCNode::EnableOperation(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    FCNode::SetOperationMode(OpMode::VelocityProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    FCNode::EnableOperation(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_control_word.enable_operation = 1;                                                // set immediately bit (bit 5)
    tpdo_mapped[0x60FF][0] = static_cast<int32_t>(this->m_vel_findlimit * this->m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get();                            // enable
    tpdo_mapped[0x60FF][0].WriteEvent();                                                // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        FCNode::GetCurrent(current);

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
            FCNode::EnableOperation(false);
            FCNode::GetActualPos(pos_1);
            this->m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_node_id << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    FCNode::SetTargetVel(0.0);
    FCNode::EnableOperation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

/*######## This function is nor validated for Faulhaber command #######*/
void FCNode::BackToZero()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    FCNode::GetActualPos(pos_0);

    FCNode::EnableOperation(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    FCNode::SetOperationMode(OpMode::PositionProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    FCNode::EnableOperation(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_control_word.enable_operation = 1;                                                // set immediately bit (bit 5)
    tpdo_mapped[0x60FF][0] = static_cast<int32_t>(this->m_vel_findlimit * this->m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get();                            // enable
    tpdo_mapped[0x60FF][0].WriteEvent();                                                // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        FCNode::GetCurrent(current);

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
            FCNode::EnableOperation(false);
            FCNode::GetActualPos(pos_1);
            this->m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_node_id << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    FCNode::SetTargetVel(0.0);
    FCNode::EnableOperation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void FCNode::SetTargetPosAbs(const double val)
{
    m_target_pos_SI = val;
    m_target_pos = static_cast<int32_t>(m_target_pos_SI * m_ppu);
}

/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void FCNode::SetTargetPos(const double val)
{
    m_target_pos_SI = val + m_pos_offset_SI;
    m_target_pos = static_cast<int32_t>(m_target_pos_SI * m_ppu);
}

/* Sets target valocit in [m/s] or [rad/s] and converts motion controller unit
    and updates the associates variable to be executes in the next sample*/
void FCNode::SetTargetVel(const double val)
{
    m_target_vel_SI = val;
    // m_target_vel = static_cast<int32_t>(m_target_vel_SI * m_ppu);
    m_target_vel = static_cast<int32_t>(m_target_vel_SI * 60 / m_gear_ratio / m_velfactor);
}

/* Sets in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void FCNode::SetHomeOffsetValue(const double val)
{
    this->m_pos_offset_SI = val;
}

/* accessor to motor current [A]*/
void FCNode::GetCurrent(double &currentPtr) const
{
    currentPtr = m_current_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void FCNode::GetActualPosAbs(double &actualPosPtr) const
{
    actualPosPtr = m_actual_pos_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void FCNode::GetActualPos(double &actualPosPtr) const
{
    actualPosPtr = m_actual_pos_SI - m_pos_offset_SI;
}

/* accessor to actual motor velocity [m/s] or [rad/s]*/
void FCNode::GetActualVel(double &actualVelPtr) const
{
    actualVelPtr = m_actual_vel_SI;
}

/* if target is reached */
bool FCNode::IsReached() const
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

/**/
void FCNode::ReadEncoderMemFile(const std::string &directory,
                                double &encoder_memory,
                                std::ofstream &encoder_memory_file)
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

/** This function sends Faulhaber Commands through PDO2
  @param command: Representing the Faulhaber command (must be among the commands on page 143 of the Faulhaber communication Manual document for  faulhaber command) to be sent.
  @param value: value associated with the command.
  @returns 0 on success, 1 on failure (e.g., if the command is unknown).**/
int FCNode::FaulhaberCommand(std::string command, int value)
{
    // Check if the value is in the CiA 402 dictionary
    if (faulhaberComCodeDictionary.find(command) != faulhaberComCodeDictionary.end())
    {
        // Retrieve the command code and prepare data for PDO2 transmission.
        uint8_t comcode = faulhaberComCodeDictionary[command]; // find code from the comman list
        tpdo_mapped[0x2301][0x01] = comcode;                   // set the command code
        tpdo_mapped[0x2301][0x02] = value;                     // set the value
        tpdo_mapped[0x2301][0x01].WriteEvent();                // Trigger write events for the PDO2 data.

        if (false) // Optionally, print log information for the sent command and value.
        {
            std::cout << "=> tPDO2 [" << m_node_id << "]: Command: " << command << "  \tvalue: " << std::dec << static_cast<int>(value) << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2301 << "][" << static_cast<int>(0x01) << "]: "
                      << "0x" << static_cast<int>(comcode) << std::endl;
            std::cout << "         [" << m_node_id << "]:[" << std::hex << 0x2301 << "][" << static_cast<int>(0x02) << "]: " << std::dec << static_cast<int>(value) << std::endl;
        }
        return 0; // Success
    }
    else
    {
        std::cout << "Unknown Faulhaber Command!" << std::endl;
        return 1; // Failure
    }
}

/*  This function is dedicated for Faulhaber Command and configs the information that must be transferred over tPDO3
    see page "Extended CAN function" chapter (page 69) of the communication function manual  */
int FCNode::FaulhaberTraceConfig()
{
    // if you change the transferred data, you should fix the data type in the .eds file as well. Otherwise
    // you will get EMCY error due to incorrect tPDO data type. Once you change the .eds file, you should do it
    // for "DataType=" under [2304sub1] and [2304sub2], (32byte => 0x0007, 16byte => 0x0006), also you should
    // change DefaultValue= under [1A02sub1] and [1A02sub2], (the last two number "0x23040120" means the number of bytes
    // for example "0x23040120" means 32 bytes, and "0x23040110" mean 16 bytes. first 4 number are for the PDO addres,
    // and numbner 5 and 6 are for the subindex. so only change the last two numbers (7 and 8) accordingly).
    tpdo_mapped[0x2303][0x01] = static_cast<int8_t>(0xC8); // actual position (int32)
    tpdo_mapped[0x2303][0x02] = static_cast<int8_t>(0x00); // actual velocity (int16)
    tpdo_mapped[0x2303][0x03] = static_cast<int8_t>(0x00); // Transmission without time code
    tpdo_mapped[0x2303][0x04] = static_cast<int8_t>(0x01);
    tpdo_mapped[0x2303][0x05] = static_cast<int8_t>(0x01); // frequnecy (1ms)
    tpdo_mapped[0x2303][0x05].WriteEvent();
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
    logger->debug("[Node " + m_node_id + "] FaulhaberTraceConfig");
    return 0;
}

// Getter function to retrieve the StatusWord
StatusWord FCNode::GetStatusword() const
{
    return m_status_word;
}

// Getters
bool FCNode::GetNodeFlags(Flags::FlagIndex index) const
{
    return m_flags.get(index);
}

// /* A method to enabled the driver and set the mode of operation
//     For CiA301 standard Application Layer, Node::Enable must be called in the OnConfig callback
//     For Faulhaber Command Application Layer, Node::Enable must be called in the OnBoot callback
//     the Enable method should be called in the OnConfig callback function.*/
// void FCNode::SwitchEnableMode(int8_t operationMode)
// {
//     // reset fault
//     Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x0080)); // writing on SDO
//     Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
//     m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
//     std::cout << "[Node " << node_id << ": " << m_status_word.getCiA402StatusMessage() << std::endl;

//     // shut down
//     Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x0006));
//     Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
//     m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
//     std::cout << "[Node " << node_id << ": " << m_status_word.getCiA402StatusMessage() << std::endl;

//     // switch on or disable operation
//     Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x0007));
//     Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
//     m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
//     std::cout << "[Node " << node_id << ": " << m_status_word.getCiA402StatusMessage() << std::endl;

//     // enable operation
//     Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x000F));
//     Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
//     m_status_word.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
//     std::cout << "[Node " << node_id << ": " << m_status_word.getCiA402StatusMessage() << std::endl;

//     // check if the driver is in enable operation mode
//     if ((m_status_word.statusword & 0xFF) == 0x0027) // compare upper 8 bits
//     {
//         // Set the mode of operation
//         Wait(AsyncWrite<int8_t>(MODE_OF_OPERATION_IDX, 0, static_cast<int8_t>(operationMode)));
//         int8_t modopdisp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));
//         std::cout << "[Node " << node_id << "Mode of operation set to " << int(modopdisp) << std::endl;
//     }
//     else
//     {
//         std::cout << "[Node " << node_id << "Mode of operation cannot be set because the operation is disabled " << std::endl;
//     }
// }