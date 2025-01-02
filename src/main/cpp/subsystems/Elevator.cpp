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
    , m_motor(ElevatorConstants::kAngleMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_feedforward(ElevatorConstants::kFFks, ElevatorConstants::kFFkg, ElevatorConstants::kFFkV,
                    ElevatorConstants::kFFkA)
    , Linear{1}
    , m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                   ElevatorConstants::kAngleEncoderPulsePerRev)}
    , m_elevatorSim(m_elevatorGearbox, ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::lowerLimit, ElevatorConstants::upperLimit, true, 0_m, {0.01})
{
    m_encoder.SetDistancePerPulse(0.0);
    // m_encoder.Reset();
    ElevatorConstants::m_holdHeight    = 0.0;
    ElevatorConstants::m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}

void Elevator::Periodic()
{
    // This method will be called once per scheduler run.
    switch(m_ElevatorState)
    {
        case ElevatorConstants::LIFT:
            break;
        case ElevatorConstants::LOWER:
            break;
        case ElevatorConstants::HOLD:
            break;
    }
}

void Elevator::SetHeight(double height)
{
    if(m_ElevatorState != LIFT && m_ElevatorState != LOWER)
    {
        if(GetHeight() > height)
        {
            m_ElevatorState = LOWER;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(kTolerancePos, kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else if(GetHeight() < height)
        {
            m_ElevatorState = LIFT;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(kTolerancePos, kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else
        {
            m_ElevatorState = HOLD;
        }
    }
}

void Elevator::SetSpeed(double speed)
{
    if(m_state == MANUAL_MOVING)
    {
        m_linearMotor.Set(speed);
    }
}

double Elevator::GetHeight()
{
    return m_linearEncoder.GetDistance() + m_offset;
}

bool Elevator::CheckGoal()
{
    return m_state == HOLD;
}

void Elevator::printLog() {}
void Elevator::handle_Setpoint() {}
void Elevator::Emergency_Stop() {}
void Elevator::SimulationPeriodic() {}