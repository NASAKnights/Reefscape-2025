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
#include "utils/POIGenerator.h"

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

private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    // Subsystems
    SwerveDrive m_swerveDrive;

    std::string_view baseLink = "base_link";

    std::string targetKey = "POI/TestPersist";

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

    POIGenerator m_poiGenerator;

    frc2::CommandPtr addPOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                           { return m_poiGenerator.MakePOI(); }))
                                         .IgnoringDisable(true);

    frc2::CommandPtr removePOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                              { return m_poiGenerator.RemovePOI(); }))
                                            .IgnoringDisable(true);

    // Robot Container methods
    void CreateRobot();
    void BindCommands();
    frc2::CommandPtr GetAutonomousCommand();
    void UpdateDashboard();
};
