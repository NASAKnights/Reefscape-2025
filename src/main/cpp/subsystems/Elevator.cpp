/*
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robotbuilder/advanced/robotbuilder-writing-pidsubsystem-code.html
https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1_elevator_sim.html
https://github.com/wpilibsuite/allwpilib/blob/main/sysid/src/main/native/cpp/analysis/ElevatorSim.cpp
*/

#include "subsystems/Elevator.h"

using State = frc::TrapezoidProfile<units::meters>::State;
using mps_sq_t =
    units::unit_t<units::compound_unit<units::velocity::mps, units::inverse<units::time::seconds>>>;

ElevatorSubsystem::ElevatorSubsystem()
    : // frc2::ProfiledPIDSubsystem<units::meter>(
      m_controller(
          ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD,
          frc::TrapezoidProfile<units::meter>::Constraints(ElevatorConstants::kMaxVelocity,
                                                           ElevatorConstants::kMaxAcceleration),
          ElevatorConstants::kDt),

      //, m_motor(ElevatorConstants::kMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)

      m_motorLeft(ElevatorConstants::kMotorIdLeft, rev::spark::SparkMax::MotorType::kBrushless),
      m_motorRight(ElevatorConstants::kMotorIdRight, rev::spark::SparkMax::MotorType::kBrushless),
      //, m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
      m_feedforwardElevator{ElevatorConstants::kFFks, ElevatorConstants::kFFkg,
                            ElevatorConstants::kFFkV, ElevatorConstants::kFFkA},
      m_encoderLeft{m_motorLeft.GetEncoder()},
      m_encoderRight{m_motorRight.GetEncoder()},
      m_elevatorSim(frc::DCMotor::NEO(ElevatorConstants::kNumMotors), ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::simLowerLimit, ElevatorConstants::simUpperLimit, true, 0_m,
                    {0.001})
{
    m_motorRight.SetInverted(true);

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_HeightLog = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");

    m_holdHeight = 0.0;
    m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}

void ElevatorSubsystem::HoldPosition()
{
    m_controller.SetTolerance(ElevatorConstants::kTolerancePos, ElevatorConstants::kToleranceVel);
    m_controller.SetGoal(units::meter_t{GetHeight()});
    m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}
// height in meters
void ElevatorSubsystem::SetHeight(double height)
{
    if (m_ElevatorState != ElevatorConstants::ElevatorState::LIFT &&
        m_ElevatorState != ElevatorConstants::ElevatorState::LOWER)
    {
        if (GetHeight() > height)
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::LOWER;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else if (GetHeight() < height)
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::LIFT;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
        }
    }
}

/*
void ElevatorSubsystem::SetSpeed(double speed)
{
    if(m_ElevatorState == ElevatorConstants::ElevatorState::MANUAL)
    {
        m_motor.Set(speed);
    }
}
*/

double ElevatorSubsystem::GetHeight()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        // frc::SmartDashboard::PutBoolean("isSim", true);
        return m_elevatorSim.GetPosition().value();
    }
    return (GetEncoderDistance(m_encoderLeft) + GetEncoderDistance(m_encoderRight) / 2.0).value();
}

units::meter_t ElevatorSubsystem::GetMeasurement()
{
    return units::meter_t{GetHeight()};
}
/*
bool ElevatorSubsystem::CheckGoal()
{
    return m_ElevatorState == ElevatorConstants::ElevatorState::HOLD;
}
*/
void ElevatorSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Elevator/elevatorGoal_POS", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_setpoint",
                                   m_controller.GetSetpoint().position.value());
    m_HeightLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_ElevatorState);
    m_MotorCurrentLog.Append((m_motorLeft.GetOutputCurrent() + m_motorRight.GetOutputCurrent()) / 2.0);
    m_MotorVoltageLog.Append((m_motorLeft.GetAppliedOutput() + m_motorRight.GetAppliedOutput()) / 2.0);
}
/*
void ElevatorSubsystem::Periodic()
{
    if constexpr (frc::RobotBase::IsTeleop())
    {
        TeleopPeriodic();
    }
}
*/
void ElevatorSubsystem::TeleopPeriodic()
{
    if (abs((GetEncoderDistance(m_encoderLeft) - GetEncoderDistance(m_encoderRight)).value()) > ElevatorConstants::kEmergencyTolerance.value())
    {
        Emergency_Stop();
    }

    double fb = m_controller.Calculate(units::meter_t{GetHeight()});
    units::volt_t ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
    units::volt_t v = units::volt_t{fb} + ff;
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_elevatorSim.SetInputVoltage(v);
    }
    m_motorLeft.SetVoltage(v);
    m_motorRight.SetVoltage(v);

    frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
    frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
    frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());

    // check if at goal
    if (m_ElevatorState != ElevatorConstants::ElevatorState::HOLD && m_controller.AtGoal())
    {
        m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
    }
    frc::SmartDashboard::PutNumber("/Elevator/Elevator Goal Height",
                                   m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Elevator/Elevator Actual Height", GetMeasurement().value());
    // This method will be called once per scheduler run.
    switch (m_ElevatorState)
    {
    case ElevatorConstants::LIFT:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "LIFT");
        break;
    case ElevatorConstants::LOWER:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "LOWER");
        break;
    case ElevatorConstants::MANUAL:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "MANUAL");
        break;
    case ElevatorConstants::HOLD:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLD");
        break;
    }
    printLog();
}
void ElevatorSubsystem::SimulationInit()
{
    m_simTimer.Reset();
}
void ElevatorSubsystem::Emergency_Stop()
{
    m_motorLeft.StopMotor();
    m_motorRight.StopMotor();
}

void ElevatorSubsystem::SimulationPeriodic()
{
    m_elevatorSim.Update(ElevatorConstants::kDt);
    m_simTimer.Reset();
}
void ElevatorSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
}
units::meter_t ElevatorSubsystem::GetEncoderDistance(rev::spark::SparkRelativeEncoder encoder)
{
    return encoder.GetPosition() * 2.0 * std::numbers::pi * ElevatorConstants::kElevatorDrumRadius;
}