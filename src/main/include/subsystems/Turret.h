#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
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
    MOVE,
    HOLD,
    START,
    TRACKING,
    DISABLED
  };

  const double kAngleP = 100.0;
  const double kAngleI = 0.0;
  const double kAngleD = 10.0; // 0.0001
  const double kIZone = 1.0;
  const auto kTurretVelLimit = units::degrees_per_second_t(360.0);
  const auto kTurretAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(1000); // Mech limit 27 rad/s^2(1500 degree_second_squared)
  const units::degree_t kTolerancePos = 1_deg;
  const units::degrees_per_second_t kToleranceVel = 0.5_deg_per_s;
  const int kAngleMotorId = 2;

  const auto kFFks = units::volt_t(0.23);                               // Volts static (motor)
  const auto kFFkg = units::volt_t(0.28);                               // Volts
  const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(0.79); // volts*s/rad
  const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.01); // volts*s^2/rad

  const bool kTurretEnableCurrentLimit = true;
  const int kTurretContinuousCurrentLimit = 35;
  const int kTurretPeakCurrentLimit = 60;
  const double kTurretPeakCurrentDuration = 0.1;

  const std::array<double, 2> kSimNoise = {0.0};
  const frc::DCMotor kSimMotor = frc::DCMotor::KrakenX60(1);
  const double kGearRatio = 81.0; // gear ratio for motor to arm
  const units::moment_of_inertia::kilogram_square_meter_t kmoi =
      units::moment_of_inertia::kilogram_square_meter_t(0.06742); // I = MR^2
  const units::length::meter_t kTurretRadius = units::length::meter_t(0.3048);
  const units::mass::kilogram_t kTurretMass = units::mass::kilogram_t(0.725748);
  const units::angle::radian_t kminAngle = -135_deg;
  const units::angle::radian_t kmaxAngle = 135_deg;
  const bool kGravity = false;
  const units::angle::radian_t kTurretStartAngle = units::angle::radian_t(0.0);

} // namespace ArmConstants

/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class Turret : public frc2::SubsystemBase
{
  using State = frc::TrapezoidProfile<units::degrees>::State;

public:
  Turret();
  void Periodic();
  void Emergency_Stop();
  void ChangeAngle();
  void UseOutput();
  void SimulationPeriodic();
  void Enable();
  void Disable();
  void SetAngle(double angle);
  void Zero();
  void HoldPosition();
  // void get_pigeon();
  units::degree_t GetMeasurement();
  TurretConstants::TurretState GetState();

  // units::time::second_t time_brake_released;

private:
  TurretConstants::TurretState m_TurretState;
  void printLog();
  ctre::phoenix6::hardware::TalonFX m_motor;
  frc::ArmFeedforward m_feedforward;
  wpi::log::DoubleLogEntry m_AngleLog;
  wpi::log::DoubleLogEntry m_SetPointLog;
  wpi::log::IntegerLogEntry m_StateLog;
  wpi::log::DoubleLogEntry m_MotorCurrentLog;
  wpi::log::DoubleLogEntry m_MotorVoltageLog;
  frc::Timer *m_timer;
  float Turret_Angle;

  bool speed;
  units::degree_t m_goal;

  frc::Timer m_simTimer;

  frc::sim::SingleJointedArmSim m_TurretSim;

  frc::ProfiledPIDController<units::degrees> m_controller;

  hal::SimDouble m_TurretSimVelocity;
  hal::SimDouble m_TurretSimposition;
};