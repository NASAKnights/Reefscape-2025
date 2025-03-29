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
    m_BatteryLog = wpi::log::DoubleLogEntry(log, "Robot/Battery");

    frc::SmartDashboard::PutString("POIName", "");
    frc::SmartDashboard::PutData("AddPOI", addPOICommand.get());
    frc::SmartDashboard::PutData("RemovePOI", removePOICommand.get());

    // frc::SmartDashboard::PutNumber("FrontLeftDegree", 0.0);
    // frc::SmartDashboard::PutNumber("FrontRightDegree", 0.0);
    // frc::SmartDashboard::PutNumber("BackLeftDegree", 0.0);
    // frc::SmartDashboard::PutNumber("BackRightDegree", 0.0);

    auto Po = frc::SmartDashboard::PutNumber("Note Po", 0.0);
    auto Px = frc::SmartDashboard::PutNumber("Note Px", 1);
    auto Py = frc::SmartDashboard::PutNumber("Note Py", 1);
    auto Do = frc::SmartDashboard::PutNumber("Note Do", 0.0);

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    // autoChooser.SetDefaultOption("AAA", );

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
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
    m_BatteryLog.Append(batteryShunt.GetVoltage());
}

// This function is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{
    // m_LED_Controller.DefaultAnimation();
}

void Robot::SetAutonomousCommand(std::string a)
{
    // // Elements in list, make this dynamic maybe?
    // std::string autoList[4] = {"1coral-mid", "1coral-mid", "1coral-mid", "1coral-mid"}; // CHANGE AUTOS
    // m_autonomousCommand = pathplanner::PathPlannerAuto(autoList[std::stoi(a)]).ToPtr();
    // autoStartPose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(autoList[std::stoi(a)])[0]->getPathPoses()[0];

    // frc::SmartDashboard::PutString("AUTO SELECTED", a);
}

void Robot::AutonomousInit()
{
    // m_autonomousCommand = this->GetAutonomousCommand();
    m_elevator.HoldPosition();
    // m_swerveDrive.TurnVisionOff(); // don't use vision during Auto
    auto m_autonomousCommand = autoChooser.GetSelected();
    m_CoralIntake.Intake(-0.25);
    // auto start = std::move(autoMap.at(1)).second;
    // m_autonomousCommand = std::move(std::move(autoMap.at(1)).first).ToPtr();
    m_swerveDrive.ResetPose(autoStartPose);

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
    m_wrist.HoldPosition();
    m_elevator.HoldPosition();
    /*
    if (m_wrist.GetState() != WristConstants::WristState::ZEROING)
    {
        m_wrist.SetAngle(m_wrist.GetMeasurement().value());
    }
    */

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
    // m_LED_Controller.TeleopLED();
}

void Robot::TeleopPeriodic()
{
    if (m_elevator.GetHeight() >= 0.35 && !m_pathfind.IsScheduled())
    {
        frc::SmartDashboard::PutNumber("drive/accelLim", 0.5);
    }
    else
    {
        frc::SmartDashboard::PutNumber("drive/accelLim", 4.0);
    }
}

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
    scoreClosest = frc2::CommandPtr(
        frc2::cmd::RunOnce(
            [&]()
            {
                using namespace pathplanner;
                using namespace frc;
                Pose2d currentPose = this->m_swerveDrive.GetPose();
                // Select Left or Right Branch
                frc::Transform2d offset = m_driverController.GetRawButton(7) ?
                    frc::Transform2d(0.0_m, 0.35_m, frc::Rotation2d()) :
                    frc::Transform2d(0.0_m, 0.0_m, frc::Rotation2d());

                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = Pose2d(currentPose.Translation(), Rotation2d());
                Pose2d endPos = m_poiGenerator.GetClosestPOI().TransformBy(offset);

                auto transformedEndPos = endPos.TransformBy(Transform2d(0.35_m, 0_m, 0_rad));
                std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({startPos, endPos, transformedEndPos});
                // Paths must be used as shared pointers
                auto path = std::make_shared<PathPlannerPath>(
                    waypoints,
                    std::vector<RotationTarget>({RotationTarget(0.25, endPos.Rotation())}),
                    std::vector<PointTowardsZone>(),
                    std::vector<ConstraintsZone>(),
                    std::vector<EventMarker>(),
                    PathConstraints(1_mps, 2.0_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
                    std::nullopt, // Ideal starting state can be nullopt for on-the-fly paths
                    GoalEndState(0_mps, endPos.Rotation()),
                    false
                );

                // Prevent this path from being flipped on the red alliance, since the given positions are already correct
                path->preventFlipping = true;

                m_pathfind = frc2::CommandPtr(AutoBuilder::followPath(path).Unwrap());
                m_pathfind.Schedule(); })
            .Unwrap());
    pathplanner::EventTrigger("Score L1").OnTrue(std::move(PlaceL1(&m_wrist, &m_elevator)).ToPtr());
    pathplanner::EventTrigger("Score L2").OnTrue(std::move(PlaceL2(&m_wrist, &m_elevator)).ToPtr());
    pathplanner::EventTrigger("Score L3").OnTrue(std::move(PlaceL3(&m_wrist, &m_elevator)).ToPtr());
    pathplanner::EventTrigger("Score L4").OnTrue(std::move(PlaceL4(&m_wrist, &m_elevator)).ToPtr());
    pathplanner::EventTrigger("Intake").WhileTrue(std::move(GrabCoral(&m_elevator, &m_wrist, &m_CoralIntake).ToPtr()));
    pathplanner::EventTrigger("OuttakeCoral").WhileTrue(std::move(RunCoralOuttake(&m_CoralIntake).ToPtr()));
    // pathplanner::NamedCommands::registerCommand("Vision", std::move(GoToPoint(&m_swerveDrive, &m_poiGenerator).ToPtr()));
    pathplanner::NamedCommands::registerCommand("TURN VISION OFF :(", frc2::CommandPtr(
                                                                          frc2::InstantCommand([&]
                                                                                               { return m_swerveDrive.TurnVisionOff(); })));
    pathplanner::NamedCommands::registerCommand("TURN VISION ON :)", frc2::CommandPtr(
                                                                         frc2::InstantCommand([&]
                                                                                              { return m_swerveDrive.TurnVisionOn(); })));

    pathplanner::EventTrigger("Reset").OnTrue(std::move(Reset(&m_elevator, &m_wrist).ToPtr()));

    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            // auto approach = m_driverController.GetRawButton(5);
            bool approach = 0;

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
                { m_elevator.Periodic(); },
                5_ms, 1_ms);
    AddPeriodic([this]
                { m_wrist.Periodic(); },
                10_ms, 2_ms);

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
    // m_LED_Controller.DefaultAnimation();
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

    // frc2::JoystickButton(&m_driverController, 5)
    //     .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    // frc2::JoystickButton(&m_driverController, 2)
    //     .OnTrue(frc2::CommandPtr(
    //         frc2::InstantCommand([this]
    //                              { return m_swerveDrive.SetOffsets(); })));

    // frc2::JoystickButton(&m_driverController, 3)
    // .WhileTrue(GoToPoint(&m_swerveDrive, &m_poiGenerator).ToPtr());

    frc2::JoystickButton(&m_driverController, 3)
        .OnTrue(scoreClosest.get())
        .OnFalse(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_pathfind.Cancel(); })));

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
        .OnTrue(PlaceL3(&m_wrist, &m_elevator).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());

    frc2::JoystickButton(&m_operatorController, 5)
        .WhileTrue(GrabCoralFar(&m_elevator, &m_wrist, &m_CoralIntake).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());

    frc2::JoystickButton(&m_operatorController, 6)
        .WhileTrue(GrabCoral(&m_elevator, &m_wrist, &m_CoralIntake).ToPtr())
        .OnFalse(Reset(&m_elevator, &m_wrist).ToPtr());

    // frc2::JoystickButton(&m_operatorController, 6)
    //     .WhileTrue(GrabAlgaeL3(&m_AlgaeIntake).ToPtr());
    // ^ Rewrite command to include elevator
    frc2::POVButton(&m_operatorController, 0)
        .WhileTrue(RunCoralIntake(&m_CoralIntake).ToPtr());

    // Left Trigger
    frc2::JoystickButton(&m_operatorController, 7)
        .WhileTrue(ScoreAlgae(&m_wrist, &m_AlgaeIntake, &m_elevator).ToPtr());

    // Right Trigger
    frc2::JoystickButton(&m_operatorController, 8)
        .WhileTrue(RunCoralOuttake(&m_CoralIntake).ToPtr());

    // Share
    frc2::JoystickButton(&m_operatorController, 9)
        .WhileTrue(DeployClimb(&m_climber).ToPtr());
    // Option
    frc2::JoystickButton(&m_operatorController, 10)
        .WhileTrue(ClimbCage(&m_climber).ToPtr());

    frc2::POVButton(&m_operatorController, 180) // Zero wrist
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                m_wrist.Zero();
                return;
            })));

    // frc2::POVButton(&m_operatorController, 0) // Zero wrist
    //     .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             m_elevator.Zero();
    //             return;
    //         })));
}

void Robot::DisabledPeriodic()
{
    // if (m_chooser.GetSelected() != prevAuto)
    // {
    //     SetAutonomousCommand(m_chooser.GetSelected());
    // }
    // else
    // {
    //     prevAuto = m_chooser.GetSelected();
    // }

    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    frc::SmartDashboard::PutBoolean("IsPersist", frc::SmartDashboard::IsPersistent(poiName));
}

void Robot::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Robot/Battery Amps", batteryShunt.GetVoltage());
    frc::SmartDashboard::PutNumber("Robot/PDH Total Current", m_pdh.GetTotalCurrent());
    // I don't know yet if this is the best way of doing this, I just wanted to
    // have this commented for later.
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
