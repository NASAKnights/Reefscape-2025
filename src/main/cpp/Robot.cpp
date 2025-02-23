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

    auto Po = frc::SmartDashboard::PutNumber("Note Po", 0.0);
    auto Px = frc::SmartDashboard::PutNumber("Note Px", 1);
    auto Py = frc::SmartDashboard::PutNumber("Note Py", 1);
    auto Do = frc::SmartDashboard::PutNumber("Note Do", 0.0);

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
            auto approach = m_driverController.GetRawButton(3);

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

    // frc2::JoystickButton(&m_driverController, 2)
    //     .OnTrue(frc2::CommandPtr(
    //         frc2::InstantCommand([this]
    //                              { return m_swerveDrive.SetOffsets(); })));

    frc2::JoystickButton(&m_driverController, 3)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                frc::SmartDashboard::PutBoolean("lifting elevator", true);
                m_elevator.SetHeight(ElevatorConstants::upperLimit.value());
                return;
            })));

    frc2::JoystickButton(&m_driverController, 4)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                frc::SmartDashboard::PutBoolean("lowering elevator", true);
                m_elevator.SetHeight(ElevatorConstants::lowerLimit.value());
                return;
            })));

    // --------------OPERATOR BUTTONS--------------------------------
    /* frc2::JoystickButton(&m_operatorController, 1)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this]
                                                      { return exampleCommandHere(); })));
    Example Button */

    // frc2::JoystickButton(&m_operatorController, 1)
    //     .OnTrue(new PlaceL4);

    // frc2::JoystickButton(&m_operatorController, 2)
    //     .OnTrue(new PlaceL3);

    // frc2::JoystickButton(&m_operatorController, 3)
    //     .OnTrue(new PlaceL2);

    // frc2::JoystickButton(&m_operatorController, 4)
    //     .OnTrue(new PlaceL1);

    // frc2::JoystickButton(&m_operatorController, 5)
    //     .OnTrue(new GrabAlgaeL2(m_intakeAlgae));

    // frc2::JoystickButton(&m_operatorController, 6)
    //     .OnTrue(new GrabAlgaeL3(m_intakeAlgae));

    frc2::JoystickButton(&m_operatorController, 1)
        .WhileTrue(RunAlgaeOuttake(&m_AlgaeIntake).ToPtr());

    frc2::JoystickButton(&m_operatorController, 2)
        .WhileTrue(RunAlgaeIntake(&m_AlgaeIntake).ToPtr());

    frc2::JoystickButton(&m_operatorController, 3)
        .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    frc2::JoystickButton(&m_operatorController, 4)
        .WhileTrue(RunCoralIntake(&m_CoralIntake).ToPtr());

    frc2::JoystickButton(&m_operatorController, 9)
        .WhileTrue(ClimbCage(&m_climber).ToPtr());

    frc2::JoystickButton(&m_operatorController, 10)
        .WhileTrue(DeployClimb(&m_climber).ToPtr());
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
