// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Wrist.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

WristSubsystem::WristSubsystem() : m_controller(
                                       WristConstants::kAngleP, WristConstants::kAngleI, WristConstants::kAngleD,
                                       frc::TrapezoidProfile<units::degrees>::Constraints(WristConstants::kArmVelLimit, WristConstants::kArmAccelLimit), 5_ms),
                                   m_motor(WristConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless), m_feedforward(WristConstants::kFFks, WristConstants::kFFkg, WristConstants::kFFkV,
                                                                                                                                           WristConstants::kFFkA),
                                   Linear{1},
                                   m_encoder{m_motor.GetEncoder()},

                                   m_WristSim(WristConstants::kSimMotor, WristConstants::kGearRatio, WristConstants::kmoi,
                                              WristConstants::kWristLength, WristConstants::kminAngle, WristConstants::kmaxAngle,
                                              WristConstants::kGravity, WristConstants::kWristStartAngle)
{
    m_controller.SetIZone(WristConstants::kIZone);
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.encoder.PositionConversionFactor(81.0);

    m_timer = new frc::Timer();
    m_motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    m_controller.SetTolerance(WristConstants::kControllerTolerance);
    // Start m_arm in neutral position
    // m_controller.SetGoal(State{units::degree_t(WristConstants::kWristAngleRetracted), 0_rad_per_s});

    m_WristState = WristConstants::WristState::START;

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Wrist/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Wrist/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Wrist/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorVoltage");

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
}

void WristSubsystem::SimulationPeriodic()
{
    m_WristSim.Update(5_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_WristSim.GetCurrentDraw().value());
}
void WristSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
}

void WristSubsystem::Emergency_Stop()
{
    if (Kill.Get() == true)
    {
        m_motor.StopMotor();
    }
}

bool WristSubsystem::isOverLimit()
{
    return Kill.Get();
}

void WristSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("armGoal_POS", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ARM_setpoint", m_controller.GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_WristState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}

units::degree_t WristSubsystem::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_WristSim.GetAngle();
    }

    return units::degree_t{m_encoder.GetPosition()};
}

void WristSubsystem::UseOutput()
{
    units::volt_t feedforward = m_feedforward.Calculate(m_controller.GetSetpoint().position, m_controller.GetSetpoint().velocity);
    units::volt_t fb = units::volt_t{m_controller.Calculate(units::radian_t{GetMeasurement().value()})};
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_WristSim.SetInputVoltage(units::volt_t{fb} + feedforward);
    }
    m_motor.SetVoltage(units::volt_t{fb} + feedforward);
}
void WristSubsystem::ChangeAngle()
{
}
void WristSubsystem::handle_Setpoint()
{

    frc::SmartDashboard::PutNumber("Wrist Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("Wrist Actual Angle", GetMeasurement().value());

    switch (m_WristState)
    {
    case WristConstants::START:
    {
        frc::SmartDashboard::PutString("State", "StartKick");
        break;
    }
    case WristConstants::ZEROED:
    {
        frc::SmartDashboard::PutString("State", "AngleUp");
        break;
    }
    case WristConstants::MOVINGTOGOAL:
    {
        frc::SmartDashboard::PutString("State", "AngleDown");
        if (m_controller.AtGoal())
        {
            m_WristState = WristConstants::ATGOAL;
        }
        break;
    }
    case WristConstants::ATGOAL:
    {
        frc::SmartDashboard::PutString("State", "AtGoal");
    }
    default:
    {
        frc::SmartDashboard::PutString("State", "default");
        break;
    }
    }
    // switch (m_WristState)
    // {
    // case WristConstants::STARTKICK:
    // {
    //     m_controller.SetGoal(units::angle::degree_t{WristConstants::kWristAngleExtended});
    //     m_WristState = WristConstants::FORWARD;
    //     m_timer->Restart();
    //     frc::SmartDashboard::PutString("State", "StartKick");
    //     break;
    // }
    // case WristConstants::FORWARD:
    // {
    //     // Forward state is set when Kick is in proccess
    //     //  After full kicking motion has been completed (successful angle has been reach)
    //     //  Return to backwards position
    //     frc::SmartDashboard::PutString("State", "Forward");

    //     if (m_controller.AtGoal())
    //     {
    //         m_WristState = WristConstants::BACKWARD;
    //         m_controller.SetGoal(units::angle::degree_t{WristConstants::kWristAngleExtended});
    //     }
    //     break;
    // }
    // case WristConstants::PRIMED:
    // {
    //     // Driver input Causes kicking motion going from Primed to Extended
    //     frc::SmartDashboard::PutString("State", "Primed");
    //     break;
    // }
    // case WristConstants::BACKWARD:
    // {
    //     // Return Arm to backwards set position
    //     frc::SmartDashboard::PutString("State", "Backward");

    //     if (m_controller.AtGoal())
    //     {
    //         m_WristState = WristConstants::PRIMED;
    //         m_controller.SetGoal(units::angle::degree_t{WristConstants::kWristAngleExtended});
    //     }

    //     break;
    // }
    // default:
    //     frc::SmartDashboard::PutString("State", "Default");
    //     break;
    // }
}

void WristSubsystem::Enable()
{
    using State = frc::TrapezoidProfile<units::degrees>::State;
}

void WristSubsystem::Disable()
{
    m_motor.StopMotor();
}