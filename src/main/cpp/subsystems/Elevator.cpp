/*
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robotbuilder/advanced/robotbuilder-writing-pidsubsystem-code.html
https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1_elevator_sim.html
*/

#include "subsystems/Elevator.h"

using State = frc::TrapezoidProfile<units::meters>::State;
using mps_sq_t =
    units::unit_t<units::compound_unit<units::velocity::mps, units::inverse<units::time::seconds>>>;

ElevatorSubsystem::ElevatorSubsystem()
    : frc2::ProfiledPIDSubsystem<units::meter>(frc::ProfiledPIDController<units::meter>(
          ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD,
          frc::TrapezoidProfile<units::meter>::Constraints(ElevatorConstants::kMaxVelocity,
                                                           ElevatorConstants::kMaxAcceleration),
          5_ms))
    , m_motor(ElevatorConstants::kMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_feedforward(ElevatorConstants::kFFks, ElevatorConstants::kFFkg, ElevatorConstants::kFFkV,
                    ElevatorConstants::kFFkA)
    , Linear{1}
    , m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                   ElevatorConstants::kEncoderPulsePerRev)}
    , m_elevatorSim(frc::DCMotor::NeoVortex(1), ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::lowerLimit, ElevatorConstants::upperLimit, true, 0_m, {0.01})
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_HeightLog            = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog          = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog             = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog      = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog      = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");

    // m_encoder.SetDistancePerPulse(0.0);
    //  m_encoder.Reset();
    ElevatorConstants::m_holdHeight    = 0.0;
    ElevatorConstants::m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}

void ElevatorSubsystem::Periodic()
{
    // This method will be called once per scheduler run.
    switch(ElevatorConstants::m_ElevatorState)
    {
        case ElevatorConstants::LIFT:
            break;
        case ElevatorConstants::LOWER:
            break;
        case ElevatorConstants::MANUAL:
            break;
        case ElevatorConstants::HOLD:
            break;
    }
}

void ElevatorSubsystem::SetHeight(double height)
{
    if(ElevatorConstants::m_ElevatorState != ElevatorConstants::ElevatorState::LIFT &&
       ElevatorConstants::m_ElevatorState != ElevatorConstants::ElevatorState::LOWER)
    {
        if(GetHeight() > height)
        {
            ElevatorConstants::m_ElevatorState = ElevatorConstants::ElevatorState::LOWER;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else if(GetHeight() < height)
        {
            ElevatorConstants::m_ElevatorState = ElevatorConstants::ElevatorState::LIFT;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else
        {
            ElevatorConstants::m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
        }
    }
}

void ElevatorSubsystem::SetSpeed(double speed)
{
    if(ElevatorConstants::m_ElevatorState == ElevatorConstants::ElevatorState::MANUAL)
    {
        m_motor.Set(speed);
    }
}

double ElevatorSubsystem::GetHeight()
{
    if constexpr(frc::RobotBase::IsSimulation())
    {
        return m_elevatorSim.GetPosition().value();
    }
    return m_encoder.GetPosition() + ElevatorConstants::m_offset;
}

bool ElevatorSubsystem::CheckGoal()
{
    return ElevatorConstants::m_ElevatorState == ElevatorConstants::ElevatorState::HOLD;
}

void ElevatorSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ELEVATOR_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("elevatorGoal_POS", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ELEVATOR_setpoint",
                                   GetController().GetSetpoint().position.value());
    m_HeightLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(GetController().GetSetpoint().position.value());
    m_StateLog.Append(ElevatorConstants::m_ElevatorState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}
void ElevatorSubsystem::handle_Setpoint() {}
void ElevatorSubsystem::Emergency_Stop() {}
void ElevatorSubsystem::SimulationPeriodic()
{
    m_elevatorSim.Update(5_ms);
}