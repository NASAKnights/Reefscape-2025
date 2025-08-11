// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <rev/SparkMax.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/acceleration.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
namespace TurretConstants
{
  enum TurretState
  {
    TRACKING,
    ZEROING,
    DISABLED,
  };

  const double kAngleP = 0.3;
  const double kAngleI = 0.0;
  const double kAngleD = 0.0; // 0.0001
  const double kIZone = 1.0;
  const auto kTurretVelLimit = units::degrees_per_second_t(360.0);
  const auto kTurretAccelLimit = units::degrees_per_second_squared_t(1500); // Mech limit 27 rad/s^2(1500 degree_second_squared)
  const units::degree_t kTolerancePos = 1_deg;
  const units::degrees_per_second_t kToleranceVel = 0.5_deg_per_s;
  const int kAngleMotorId = 2;

  const auto kFFks = units::volt_t(0.23);                               // Volts static (motor)
  const auto kFFkg = units::volt_t(0.28);                               // Volts
  const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(0.79); // volts*s/rad
  const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.01); // volts*s^2/rad

  const bool kWristEnableCurrentLimit = true;
  const int kWristContinuousCurrentLimit = 35;
  const int kWristPeakCurrentLimit = 60;
  const double kWristPeakCurrentDuration = 0.1;

  double targetangle = 0.0;

}
class Turret
{
public:
  Turret();
  void Periodic();
  void SimulationPeriodic();
  void Enable();
  void Disable();
  void GetTurretAngle();
  void GetTurretSpeed();
  void GetTargetAngle();
  void SetTurretAngle(double targetAngle);
  void printLog();

  TurretConstants::TurretState m_turretState;
  rev::spark::SparkMax m_motor;
  frc::ArmFeedforward m_feedforward;
  wpi::log::DoubleLogEntry m_AngleLog;
  wpi::log::DoubleLogEntry m_SetPointLog;
  wpi::log::IntegerLogEntry m_StateLog;
  wpi::log::DoubleLogEntry m_MotorCurrentLog;
  wpi::log::DoubleLogEntry m_MotorVoltageLog;
  frc::ProfiledPIDController<units::degrees> m_controller;
  rev::spark::SparkRelativeEncoder m_encoder;
};
