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

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>

namespace WristConstants
{

    enum WristState
    {
        MOVINGTOGOAL,
        ATGOAL,
        START,
        ZEROED,
    };

    const double kAngleP = 0.3;
    const double kAngleI = 0.0;
    const double kAngleD = 0.0; // 0.0001
    const double kIZone = 0.0;
    const auto kArmVelLimit = units::degrees_per_second_t(90.0);
    const auto kArmAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(
        90); // Mech limit 27 rad/s^2(1500 degree_second_squared)
    const auto kControllerTolerance = units::degree_t(2.0);
    const int kAngleMotorId = 5;

    const int kAngleEncoderPulsePerRev = 42;
    const auto kFFks = units::volt_t(0.23);                               // Volts static (motor)
    const auto kFFkg = units::volt_t(0.28);                               // Volts
    const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(0.79); // volts*s/rad
    const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.01); // volts*s^2/rad

    const bool kWristEnableCurrentLimit = true;
    const int kWristContinuousCurrentLimit = 35;
    const int kWristPeakCurrentLimit = 60;
    const double kWristPeakCurrentDuration = 0.1;

    const std::array<double, 1> kSimNoise = {0.0087};
    const frc::DCMotor kSimMotor = frc::DCMotor::NEO550(1);

    const double kWristAngleStraight = 0.0; // With offset
    const double kWristAngleUP = 45;        // With offset
    const double kWristAngleDown = -45.0;   // with offset
    const double kMaxTimer = 2.0;           // Max time for kicking (seconds)
    const units::angle::radian_t kTolerance = 0.0349066_rad;

    const double kGearRatio = 81.0; // gear ratio for motor to arm
    const units::moment_of_inertia::kilogram_square_meter_t kmoi =
        units::moment_of_inertia::kilogram_square_meter_t(0.00902);
    const units::length::meter_t kWristLength = units::length::meter_t(0.1778);
    const units::angle::radian_t kminAngle = units::angle::radian_t(-0.7854);
    const units::angle::radian_t kmaxAngle = units::angle::radian_t(0.7854);
    const bool kGravity = true;
    const units::angle::radian_t kWristStartAngle = units::angle::radian_t(0.0);

} // namespace ArmConstants

/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class WristSubsystem : public frc2::SubsystemBase
{
    using State = frc::TrapezoidProfile<units::degrees>::State;

public:
    WristSubsystem();
    void printLog();
    void handle_Setpoint();
    void Emergency_Stop();
    void ChangeAngle();
    void UseOutput();
    void SimulationPeriodic();
    void Enable();
    void Disable();
    // void get_pigeon();
    void UseOutput(double output, State setpoint);
    units::degree_t GetMeasurement();
    bool isOverLimit();

    units::time::second_t time_brake_released;
    WristConstants::WristState m_WristState;

private:
    rev::spark::SparkMax m_motor;
    frc::ArmFeedforward m_feedforward;
    wpi::log::DoubleLogEntry m_AngleLog;
    wpi::log::DoubleLogEntry m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;
    ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9}; //, "NKCANivore"};
    frc::Timer *m_timer;
    rev::spark::SparkRelativeEncoder m_encoder;
    float Wrist_Angle;
    frc::PWM Linear;
    frc::DigitalInput Kill{4};

    frc::Timer m_simTimer;

    frc::sim::SingleJointedArmSim m_WristSim;

    frc::ProfiledPIDController<units::degrees> m_controller;

    hal::SimDouble m_WristSimVelocity;
    hal::SimDouble m_WristSimposition;
};