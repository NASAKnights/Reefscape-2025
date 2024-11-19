// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Arm.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

ArmSubsystem::ArmSubsystem()
    : frc2::ProfiledPIDSubsystem<units::degrees>(frc::ProfiledPIDController<units::degrees>(
          ArmConstants::kAngleP, ArmConstants::kAngleI, ArmConstants::kAngleD,
          frc::TrapezoidProfile<units::degrees>::Constraints(ArmConstants::kArmVelLimit,
                                                             ArmConstants::kArmAccelLimit),
          5_ms))
    , m_motor(ArmConstants::kAngleMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_feedforward(ArmConstants::kFFks, ArmConstants::kFFkg, ArmConstants::kFFkV,
                    ArmConstants::kFFkA)
    , Linear{1} // arm_pigeon{9, "NKCANivore"}
{
    auto armAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();
    GetController().SetIZone(ArmConstants::kIZone);
    auto encoder = m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                      ArmConstants::kAngleEncoderPulsePerRev);
    encoder.SetPositionConversionFactor(6.0);

    armAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentLimit     = ArmConstants::kArmContinuousCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyTimeThreshold    = ArmConstants::kArmPeakCurrentDuration;

    // m_motor.GetConfigurator().Apply(armAngleConfig);
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start m_arm in neutral position
    SetGoal(State{units::degree_t(0.0), 0_rad_per_s});

    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_AngleLog             = wpi::log::DoubleLogEntry(log, "/Arm/Angle");
    m_SetPointLog          = wpi::log::DoubleLogEntry(log, "/Arm/Setpoint");
    m_StateLog             = wpi::log::IntegerLogEntry(log, "/Arm/State");
    m_MotorCurrentLog      = wpi::log::DoubleLogEntry(log, "/Arm/MotorCurrent");
    m_MotorVoltageLog      = wpi::log::DoubleLogEntry(log, "/Arm/MotorVoltage");
}

void ArmSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
    units::volt_t feedforward = m_feedforward.Calculate(setpoint.position, setpoint.velocity);
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
}

void ArmSubsystem::Emergency_Stop()
{
    if(Kill.Get() == true)
    {
        m_motor.StopMotor();
    }
}

bool ArmSubsystem::isOverLimit()
{
    return Kill.Get();
}

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("armGoal_POS", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ARM_setpoint", GetController().GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(GetController().GetSetpoint().position.value());
    m_StateLog.Append(m_ArmState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}

units::degree_t ArmSubsystem::GetMeasurement()
{ // original get measurement function
    return units::degree_t{m_motor
                               .GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature,
                                           ArmConstants::kAngleEncoderPulsePerRev)
                               .GetPosition()};
}

// units::degree_t ArmSubsystem::GetMeasurement() { //with redunant encoders
//   units::degree_t avrage_encoder =
//   (units::degree_t{(m_encoderL.GetDistance())}+units::degree_t{(m_encoderR.GetDistance())})/2.0;
//   if(m_encoderL.GetDistance() == -ArmConstants::kArmAngleOffsetL){
//    return units::degree_t{(m_encoderR.GetDistance())};
//   }
//  if(m_encoderR.GetDistance() == -ArmConstants::kArmAngleOffsetR){
//    return units::degree_t{(m_encoderL.GetDistance())};
//   }
//   return avrage_encoder;
// }
void ArmSubsystem::kick()
{
    if(ArmConstants::PRIMED == m_ArmState)
    {
        ArmSubsystem::m_ArmState = ArmConstants::STARTKICK;
    }
}
void ArmSubsystem::handle_Setpoint(units::angle::degree_t setpoint)
{

    switch(m_ArmState)
    {
        case ArmConstants::STARTKICK:
        {
            SetGoal(units::angle::dergee_t{ArmConstants::kArmAngleExtended});
            m_ArmState = ArmConstants::FORWARD;
        }
        case ArmConstants::FORWARD:
        {
            // Forward state is set when Kick is in proccess
            //  After full kicking motion has been completed (successful angle has been reach)
            //  Return to backwards position
            if(GetController().AtGoal())
            {
                m_ArmState = ArmConstants::BACKWARD;
                SetGoal(units::angle::degree_t{ArmConstants::kArmAngleRetracted});
            }
            break;
        }
        case ArmConstants::PRIMED:
        {
            // Driver input Causes kicking motion going from Primed to Extended
            break;
        }
        case ArmConstants::BACKWARD:
        {
            // Return Arm to backwards set position
            if(GetController().AtGoal())
            {
                m_ArmState = ArmConstants::PRIMED;
                SetGoal(units::angle::degree_t{ArmConstants::kArmAngleExtended});
            }
            break;
        }
        default:
            break;
    }
}
