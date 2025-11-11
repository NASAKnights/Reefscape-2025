// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <units/velocity.h>
#include <rev/SparkBase.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Turret_Shooter : public frc2::SubsystemBase
{
public:
  Turret_Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void StopMotors();
  void SetSpeed();
  double max_speed = 5000; // need to change to actual value we want
  double min_speed = -5000;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkFlex m_mainShooterMotor{13, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex m_followerShooterMotor{14, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_backMotor{15, rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkBaseConfig followerShooterMotorConfig;
  rev::spark::SparkBaseConfig mainShooterMotorConfig;
  rev::spark::SparkBaseConfig backShooterMotorConfig;

  rev::spark::SparkClosedLoopController mainMotorController = m_mainShooterMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController backMotorController = m_backMotor.GetClosedLoopController();

  double shooterSpeed = 0.0;

  double kP = 0.005;
  double kI = 0.0;
  double kD = 0.0;
  double kMinOutput = -1.0;
  double kMaxOutput = 1.0;
};
