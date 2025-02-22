// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

Robot::Robot()
{
    this->CreateRobot();
}

// This function is called during startup
void Robot::RobotInit()
{
    frc::DataLogManager::Start();
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_VoltageLog = wpi::log::DoubleLogEntry(log, "/PDP/Voltage");
    m_CurrentLog = wpi::log::DoubleLogEntry(log, "/PDP/Current");
    m_PowerLog = wpi::log::DoubleLogEntry(log, "/PDP/Power");
    m_EnergyLog = wpi::log::DoubleLogEntry(log, "/PDP/Energy");
    m_TemperatureLog = wpi::log::DoubleLogEntry(log, "/PDP/Temperature");
};

// This function is called every 20 ms
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    this->UpdateDashboard();
    m_VoltageLog.Append(m_pdh.GetVoltage());
    m_CurrentLog.Append(m_pdh.GetTotalCurrent());
    m_PowerLog.Append(m_pdh.GetTotalPower());
    m_EnergyLog.Append(m_pdh.GetTotalEnergy());
    m_TemperatureLog.Append(m_pdh.GetTemperature());
}

// This function is called once each time the robot enters Disabled mode.
void Robot::DisabledInit() {}

void Robot::AutonomousInit()
{
    // m_autonomousCommand = this->GetAutonomousCommand();
    m_swerveDrive.TurnVisionOff(); // don't use vision during Auto

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_wrist.Zero();

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
}

void Robot::TeleopPeriodic()
{
    m_wrist.handle_Setpoint(26);
}

void Robot::TeleopExit()
{
    m_wrist.Disable();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot()
{
    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            auto leftXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto leftYAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto rightXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(2),
                                                        DriveConstants::kDefaultAxisDeadband);

            m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                -rightXAxis * DriveConstants::kMaxRotationalVelocity, m_swerveDrive.GetHeading()));
        },
        {&m_swerveDrive}));

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands()
{

    // --------------DRIVER BUTTONS----------------------------------
    frc2::JoystickButton(&m_driverController, 1)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_swerveDrive.ResetHeading(); })));

    frc2::JoystickButton(&m_driverController, 2)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_swerveDrive.SetOffsets(); })));

    frc2::JoystickButton(&m_driverController, 7)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                frc::SmartDashboard::PutBoolean("kicking", true);
                // m_wrist.kick();
                return;
            })))
        .OnFalse((frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                frc::SmartDashboard::PutBoolean("kicking", false);
                return;
            }))));

    // --------------OPERATOR BUTTONS--------------------------------
    /* frc2::JoystickButton(&m_operatorController, 1)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this]
                                                      { return exampleCommandHere(); })));
    Example Button */
}

void Robot::DisabledPeriodic() {}

void Robot::UpdateDashboard()
{
    // frc::ShuffleboardLayout& autoCommands =
    //     frc::Shuffleboard::GetTab("Auto Config").GetLayout("Auto Selection",
    //                                                        frc::BuiltInLayouts::kList);
    // autoCommands.Add("NAME", commandName);

    // I don't know yet if this is the best way of doing this, I just wanted to
    // have this commented for later.
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
