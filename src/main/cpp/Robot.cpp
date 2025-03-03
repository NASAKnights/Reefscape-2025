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

    frc::SmartDashboard::PutString("POIName", "");
    frc::SmartDashboard::PutData("AddPOI", addPOICommand.get());
    frc::SmartDashboard::PutData("RemovePOI", removePOICommand.get());

    // auto Po = frc::SmartDashboard::PutNumber("Note Po", 0.0);
    // auto Px = frc::SmartDashboard::PutNumber("Note Px", 1);
    // auto Py = frc::SmartDashboard::PutNumber("Note Py", 1);
    // auto Do = frc::SmartDashboard::PutNumber("Note Do", 0.0);

    std::string testAutoCalibration = "2mForward";
    auto a4 = pathplanner::PathPlannerAuto(testAutoCalibration);
    auto a4Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(testAutoCalibration)[0]->getPathPoses()[0];
    auto entry4 = std::make_pair(std::move(a4), a4Pose);
    autoMap.emplace(1, std::move(entry4));
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
void Robot::DisabledInit()
{
    m_LED_Controller.DefaultAnimation();
}

void Robot::AutonomousInit()
{
    // m_autonomousCommand = this->GetAutonomousCommand();
    m_elevator.HoldPosition();
    m_swerveDrive.TurnVisionOff(); // don't use vision during Auto

    auto start = std::move(autoMap.at(1)).second;
    m_autonomousCommand = std::move(std::move(autoMap.at(1)).first).ToPtr();
    m_swerveDrive.ResetPose(start);

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
    m_elevator.Disable();
}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_elevator.HoldPosition();
    if (m_wrist.GetState() != WristConstants::WristState::ZEROING)
    {
        m_wrist.SetAngle(m_wrist.GetMeasurement().value());
    }

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
    m_LED_Controller.TeleopLED();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit()
{
    m_elevator.Disable();
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
            auto approach = m_driverController.GetRawButton(5);

            auto leftXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto leftYAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto rightXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(2),
                                                        DriveConstants::kDefaultAxisDeadband);

            m_swerveDrive.WeightedDriving(approach, leftXAxis, leftYAxis, rightXAxis, targetKey);

            // m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            //     -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
            //     -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
            //     -rightXAxis * DriveConstants::kMaxRotationalVelocity, m_swerveDrive.GetHeading()));
        },
        {&m_swerveDrive}));

    AddPeriodic([this]
                { m_elevator.Periodic(); }, 5_ms, 1_ms);
    AddPeriodic([this]
                { m_wrist.Periodic(); }, 10_ms, 2_ms);

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
    m_LED_Controller.DefaultAnimation();
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

    frc2::JoystickButton(&m_driverController, 5)
        .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    // frc2::JoystickButton(&m_driverController, 2)
    //     .OnTrue(frc2::CommandPtr(
    //         frc2::InstantCommand([this]
    //                              { return m_swerveDrive.SetOffsets(); })));

    // frc2::JoystickButton(&m_driverController, 3)
    //     .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             frc::SmartDashboard::PutBoolean("lifting elevator", true);
    //             m_elevator.SetHeight(ElevatorConstants::upperLimit.value());
    //             return;
    //         })));

    // frc2::JoystickButton(&m_driverController, 4)
    //     .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             frc::SmartDashboard::PutBoolean("lowering elevator", true);
    //             m_elevator.SetHeight(ElevatorConstants::lowerLimit.value());
    //             return;
    //         })));

    // frc2::JoystickButton(&m_driverController, 7)
    //     .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             m_wrist.SetAngle(25);
    //             return;
    //         })))
    //     .OnFalse((frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             m_wrist.SetAngle(60);
    //             return;
    //         }))));

    frc2::JoystickButton(&m_driverController, 10)
        .WhileTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                m_climber.Unspool();
                return;
            })))
        .OnFalse(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                m_climber.Stop();
                return;
            })));

    // --------------OPERATOR BUTTONS--------------------------------
    /* frc2::JoystickButton(&m_operatorController, 1)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this]
                                                      { return exampleCommandHere(); })));
    Example Button */

    // Square
    frc2::JoystickButton(&m_operatorController, 1)
        .OnTrue(PlaceL4(&m_wrist, &m_elevator).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());
    // Cross
    frc2::JoystickButton(&m_operatorController, 2)
        .OnTrue(PlaceL1(&m_wrist, &m_elevator).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());
    // Circle
    frc2::JoystickButton(&m_operatorController, 3)
        .OnTrue(PlaceL2(&m_wrist, &m_elevator).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());
    // Triangle
    frc2::JoystickButton(&m_operatorController, 4)
        .OnTrue(PlaceL3(&m_wrist, &m_elevator, &m_AlgaeIntake).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());

    frc2::JoystickButton(&m_operatorController, 5)
        .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    // frc2::JoystickButton(&m_operatorController, 6)
    //     .WhileTrue(GrabAlgaeL3(&m_AlgaeIntake).ToPtr());
    // ^ Rewrite command to include elevator
    frc2::POVButton(&m_operatorController, 180)
        .OnTrue(GrabCoral(&m_elevator, &m_wrist, &m_CoralIntake).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());

    // Left Trigger
    frc2::JoystickButton(&m_operatorController, 7)
        .WhileTrue(RunAlgaeOuttake(&m_AlgaeIntake).ToPtr());

    // Right Trigger
    frc2::JoystickButton(&m_operatorController, 8)
        .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    // Share
    frc2::JoystickButton(&m_operatorController, 9)
        .WhileTrue(DeployClimb(&m_climber).ToPtr());
    // Option
    frc2::JoystickButton(&m_operatorController, 10)
        .WhileTrue(ClimbCage(&m_climber).ToPtr());
}

void Robot::DisabledPeriodic()
{

    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    frc::SmartDashboard::PutBoolean("IsPersist", frc::SmartDashboard::IsPersistent(poiName));
}

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
