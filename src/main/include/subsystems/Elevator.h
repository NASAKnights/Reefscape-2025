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
#include <rev/CANSparkFlex.h>
#include <units/angle.h>
#include <units/time.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/SimDeviceSim.h>

namespace ElevatorConstants
{

enum ElevatorState
{
    LIFT,
    LOWER,
    HOLD
};

const double                                        upperLimit       = 10; // in meters
const double                                        lowerLimit       = 0;  // in meters
static constexpr units::meters_per_second_t         kMaxVelocity     = 0.1_mps;
static constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.025_mps_sq;
static constexpr double                             kP               = 10.0;
static constexpr double                             kI               = 10.0;
static constexpr double                             kD               = 0.0;
static constexpr units::volt_t                      kS = 0.2_V; // minimum voltage to move motor
static constexpr auto                               kV = 0.0_V / 0.28_mps;
static constexpr auto                               kA = 0.0_V / 0.28_mps_sq;

static constexpr units::meter_t             kTolerancePos = 0.001_m;
static constexpr units::meters_per_second_t kToleranceVel = 0.001_mps;

const auto kFFks = units::volt_t(0.23);                                    // Volts static (motor)
const auto kFFkg = units::volt_t(0.28);                                    // Volts
const auto kFFkV = units::unit_t<frc::ElevatorFeedforward::kv_unit>(1.01); // volts*s/rad
const auto kFFkA = units::unit_t<frc::ElevatorFeedforward::ka_unit>(0.01); // volts*s^2/rad

static constexpr units::second_t kDt = 20_ms;

frc::TrapezoidProfile<units::meters>::Constraints m_constraints{kMaxVelocity, kMaxAcceleration};

frc::ProfiledPIDController<units::meters>  m_controller{kP, kI, kD, m_constraints, kDt};
frc::SimpleMotorFeedforward<units::meters> m_feedforward{kS, kV, kA};

double m_holdHeight;

ElevatorConstants::ElevatorState m_ElevatorState;

double m_elevatorGearbox;
double kElevatorGearing;
double kCarriageMass;
double kElevatorDrumRadius;
}

class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::degrees>::State;

public:
    ElevatorSubsystem();
    void printLog();
    void handle_Setpoint();
    void Emergency_Stop();
    void SimulationPeriodic();
    void GetHeight();
    void SetSpeed(double speed);
    void SetHeight(double height);
    void CheckGoal();

private:
    rev::CANSparkFlex                 m_motor;
    frc::ArmFeedforward               m_feedforward;
    wpi::log::DoubleLogEntry          m_HeightLog;
    wpi::log::DoubleLogEntry          m_SetPointLog;
    wpi::log::IntegerLogEntry         m_StateLog;
    wpi::log::DoubleLogEntry          m_MotorCurrentLog;
    wpi::log::DoubleLogEntry          m_MotorVoltageLog;
    ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9}; //, "NKCANivore"};
    frc::Timer*                       m_timer;
    frc::PWM                          Linear;
    rev::SparkRelativeEncoder         m_encoder;

    frc::Timer m_simTimer;

    frc::sim::ElevatorSim m_elevatorSim;
};