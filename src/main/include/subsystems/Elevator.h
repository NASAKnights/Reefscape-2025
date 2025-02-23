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
// #include <rev/CANSparkMax.h>
#include <rev/SparkMax.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/time.h>
#include <math.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/controller/ProfiledPIDController.h>

namespace ElevatorConstants
{

    enum ElevatorState
    {
        DISABLED,
        START_HOLD,
        HOLDING,
        START_MOVE,
        MOVING
    };

    static constexpr units::meter_t upperLimit = 3_in; // 57
    static constexpr units::meter_t lowerLimit = 1_in;
    static constexpr units::meter_t simUpperLimit = 57.1_in;
    static constexpr units::meter_t simLowerLimit = -0.1_in;

    static constexpr units::meters_per_second_t kMaxVelocity = 1.0_in / 1_s;                     // 61.55
    static constexpr units::meters_per_second_squared_t kMaxAcceleration = 1.0_in / (1_s * 1_s); // 460_in / (1_s * 1_s);
    static constexpr double kP = 1.0;                                                            // 0.6 - 15
    static constexpr double kI = 0.3;                                                            // 0.0

    static constexpr double kD = 0.0;
    static constexpr units::volt_t kS = 0.2_V; // minimum voltage to move motor

    static constexpr units::meter_t kTolerancePos = 0.01_m;
    static constexpr units::meters_per_second_t kToleranceVel = 0.05_mps;

    static constexpr units::meter_t kEmergencyTolerance = 0.25_in;

    const int kMotorIdLeft = 5;
    const int kMotorIdRight = 6;
    // const int kEncoderPulsePerRev = 42;

    static constexpr auto kFFks = 0.4_V;                  // Volts static (motor)
    static constexpr auto kFFkg = 0.0_V;                  // 0.54_V;                         // Volts
    static constexpr auto kFFkV = 0.5 * 2.68_V / 1.0_mps; // volts*s/meters //1.01 // 2.23
    static constexpr auto kFFkA = 0.10_V / 1.0_mps_sq;    // volts*s^2/meters //0.1

    static constexpr units::second_t kDt = 20_ms;

    // number of motors
    static constexpr int kNumMotors = 2;
    // gearing between motor and drum
    static constexpr double kElevatorGearing = 5;
    // effective carriage mass: carriage mass = m
    // maths: 1 state = 1/1 * m1, 2 stage = 1/2 * m1 + 2/2 * m2, 3 stage = 1/3 * m1 + 2/3 * m2 + 3/3 * m3
    // highest number stage = carriage

    // static constexpr units::kilogram_t kCarriageMass = (17.5_lb + 0.5 * 5_lb);
    static constexpr units::kilogram_t kCarriageMass = (13_lb);
    // effective drum radius = radius of first stage * number of stages
    // pulses per rev must be set correctly in the SparkMax encoder - only reports revolutions
    static constexpr units::meter_t kElevatorDrumRadius = 1.432_in * 2;
}

class ElevatorSubsystem : public frc2::SubsystemBase
// class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::meter>::State;

public:
    ElevatorSubsystem();
    void printLog();
    // void Emergency_Stop();
    void SimulationPeriodic();
    void SimulationInit();
    double GetHeight();
    void UseOutput(double output, State setpoint);
    units::meter_t GetMeasurement();
    void HoldPosition();
    // void           SetSpeed(double speed);
    void SetHeight(double height);
    // bool           CheckGoal();
    void Periodic();
    void TeleopInit();
    void AutonomousInit();
    void Disable();
    bool IsHolding();
    units::meter_t GetEncoderDistance(rev::spark::SparkRelativeEncoder);

private:
    frc::ProfiledPIDController<units::meter> m_controller;
    // frc::PIDController m_holdController;

    rev::spark::SparkMax m_motorLeft;
    rev::spark::SparkMax m_motorRight;

    frc::ElevatorFeedforward m_feedforwardElevator;

    wpi::log::DoubleLogEntry m_HeightLog;
    wpi::log::DoubleLogEntry m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;

    rev::spark::SparkRelativeEncoder m_encoderLeft;
    rev::spark::SparkRelativeEncoder m_encoderRight;

    frc::Timer m_simTimer;
    frc::Timer *m_timer;

    frc::sim::ElevatorSim m_elevatorSim;

    double m_holdHeight;

    ElevatorConstants::ElevatorState m_ElevatorState;

    units::meter_t m_goal;
};