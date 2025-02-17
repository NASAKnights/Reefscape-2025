// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <optional>

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/SwerveDrive.hpp"
#include "subsystems/Elevator.h"

#include "Commands/RunCoralIntake.h"
#include "Commands/RunCoralOuttake.h"
#include "Commands/RunAlgaeIntake.h"
#include "Commands/RunAlgaeOuttake.h"
#include "Commands/PlaceL4.h"
#include "Commands/PlaceL3.h"
#include "Commands/PlaceL2.h"
#include "Commands/PlaceL1.h"
#include "commands/GrabAlgaeL2.h"
#include "commands/GrabAlgaeL3.h"
#include "commands/ScoreAlgae.h"
#include "commands/DeployClimber.h"
#include "commands/ClimbCage.h"
#include "subsystems/IntakeAlgae.h"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <cmath>

class Robot : public frc::TimedRobot
{
public:
    Robot();

    //
    // Robot Schedule methods
    //
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

    // For Testing

private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    std::map<int, std::pair<pathplanner::PathPlannerAuto, frc::Pose2d>> autoMap;

    // Subsystems

    RunCoralIntake *m_runCoralIntake;
    RunCoralOuttake *m_runCoralOuttake;
    RunAlgaeIntake *m_runAlgaeIntake;
    RunAlgaeOuttake *m_runAlgaeOuttake;
    GrabAlgaeL2 *m_grabAlgaeL2;
    GrabAlgaeL3 *m_grabAlgaeL3;
    IntakeAlgae *m_intakeAlgae;

    DeployClimber deployClimber;
    ClimbCage climbCage;
    ScoreAlgae scoreAlgae;
    SwerveDrive m_swerveDrive;
    ElevatorSubsystem m_elevator;

    frc::PowerDistribution m_pdh =
        frc::PowerDistribution{1, frc::PowerDistribution::ModuleType::kRev};

    // PS4 controllers
    frc::Joystick m_driverController{DriveConstants::kDriverPort};
    frc::Joystick m_operatorController{DriveConstants::kOperatorPort};

    // Power Distribution
    wpi::log::DoubleLogEntry m_VoltageLog;
    wpi::log::DoubleLogEntry m_CurrentLog;
    wpi::log::DoubleLogEntry m_PowerLog;
    wpi::log::DoubleLogEntry m_EnergyLog;
    wpi::log::DoubleLogEntry m_TemperatureLog;

    // Robot Container methods
    void CreateRobot();
    void BindCommands();
    frc2::CommandPtr GetAutonomousCommand();
    void UpdateDashboard();
};
