#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <rev/CANSparkFlex.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
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
    HOLD,
    MANUAL
};

const auto                                          upperLimit       = 10_m;
const auto                                          lowerLimit       = 0_m;
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

const int kMotorId            = 5;
const int kEncoderPulsePerRev = 42;

const auto kFFks = units::volt_t(0.23); // Volts static (motor)
const auto kFFkg = units::volt_t(0.28); // Volts
const auto kFFkV = 1.01_V / 1.0_mps;    // volts*s/rad
const auto kFFkA = 0.01_V / 1.0_mps_sq; // volts*s^2/rad

static constexpr units::second_t kDt = 20_ms;

frc::TrapezoidProfile<units::meters>::Constraints m_constraints{kMaxVelocity, kMaxAcceleration};

frc::ProfiledPIDController<units::meters>  m_controller{kP, kI, kD, m_constraints, kDt};
frc::SimpleMotorFeedforward<units::meters> m_feedforwardElevator{kS, kV, kA};

double m_holdHeight;

ElevatorConstants::ElevatorState m_ElevatorState;

double kElevatorGearing    = 5;
auto   kCarriageMass       = 10_kg;
auto   kElevatorDrumRadius = 0.1_m;
}

class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::meter>::State;

public:
    ElevatorSubsystem();
    void           printLog();
    void           handle_Setpoint();
    void           Emergency_Stop();
    void           SimulationPeriodic();
    double         GetHeight();
    void           UseOutput(double output, State setpoint) override;
    units::meter_t GetMeasurement() override;
    void           SetSpeed(double speed);
    void           SetHeight(double height);
    bool           CheckGoal();
    void           Periodic();

private:
    rev::CANSparkFlex                 m_motor;
    frc::ElevatorFeedforward          m_feedforwardElevator;
    wpi::log::DoubleLogEntry          m_HeightLog;
    wpi::log::DoubleLogEntry          m_SetPointLog;
    wpi::log::IntegerLogEntry         m_StateLog;
    wpi::log::DoubleLogEntry          m_MotorCurrentLog;
    wpi::log::DoubleLogEntry          m_MotorVoltageLog;
    ctre::phoenix6::hardware::Pigeon2 elevator_pigeon{9}; //, "NKCANivore"};
    frc::Timer*                       m_timer;
    frc::PWM                          Linear;
    rev::SparkRelativeEncoder         m_encoder;

    frc::Timer m_simTimer;

    frc::sim::ElevatorSim m_elevatorSim;
    double                m_offset = 0.0;
};