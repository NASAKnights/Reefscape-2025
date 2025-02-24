// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Wrist.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Wrist::Wrist() : m_controller(
                     WristConstants::kAngleP, WristConstants::kAngleI, WristConstants::kAngleD,
                     frc::TrapezoidProfile<units::degrees>::Constraints(WristConstants::kArmVelLimit, WristConstants::kArmAccelLimit), 5_ms),
                 m_motor(WristConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless), m_feedforward(WristConstants::kFFks, WristConstants::kFFkg, WristConstants::kFFkV,
                                                                                                                         WristConstants::kFFkA),
                 Linear{1},
                 m_encoder{m_motor.GetEncoder()},

                 m_WristSim(WristConstants::kSimMotor, WristConstants::kGearRatio, WristConstants::kmoi,
                            WristConstants::kWristLength, WristConstants::kminAngle, WristConstants::kmaxAngle,
                            WristConstants::kGravity, WristConstants::kWristStartAngle),
                 Zeroed{6}
{
    m_controller.SetIZone(WristConstants::kIZone);
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.encoder.PositionConversionFactor(81.0);

    m_motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    m_controller.SetTolerance(WristConstants::kTolerancePos, WristConstants::kToleranceVel);
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

void Wrist::SimulationPeriodic()
{
    m_WristSim.Update(5_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_WristSim.GetCurrentDraw().value());
}
void Wrist::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
}

void Wrist::Emergency_Stop()
{
    if (Kill.Get() == true)
    {
        m_motor.StopMotor();
    }
}

bool Wrist::isOverLimit()
{
    return Kill.Get();
}

units::degree_t Wrist::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_WristSim.GetAngle();
    }

    return units::degree_t{m_encoder.GetPosition()};
}

void Wrist::UseOutput()
{
    units::volt_t feedforward = m_feedforward.Calculate(m_controller.GetSetpoint().position, m_controller.GetSetpoint().velocity);
    units::volt_t fb = units::volt_t{m_controller.Calculate(units::radian_t{GetMeasurement().value()})};
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_WristSim.SetInputVoltage(units::volt_t{fb} + feedforward);
    }
    m_motor.SetVoltage(units::volt_t{fb} + feedforward);
}
void Wrist::ChangeAngle() {}

void Wrist::SetAngle(double wristAngleGoal)
{

    if (m_WristState != WristConstants::MOVE &&
        m_WristState != WristConstants::ZEROED)
    {
        m_goal = units::degree_t{wristAngleGoal};
        if (GetMeasurement().value() >= wristAngleGoal)
        {
            m_WristState = WristConstants::MOVE;
            // m_controller.Reset(GetMeasurement());
            // m_controller.SetTolerance(WristConstants::kTolerancePos,
            //   WristConstants::kToleranceVel);
            // m_controller.SetGoal(units::degree_t{wristAngleGoal});
        }
        else if (GetMeasurement().value() <= wristAngleGoal)
        {
            m_WristState = WristConstants::MOVE;
            // m_controller.Reset(GetMeasurement());
            // m_controller.SetTolerance(WristConstants::kTolerancePos,
            //                           WristConstants::kToleranceVel);
            // m_controller.SetGoal(units::degree_t{wristAngleGoal});
        }
        else
        {
            m_WristState = WristConstants::HOLD;
        }
    }
}

void Wrist::HandleSetpoint(int wristAngleGoal)
{
    frc::SmartDashboard::PutNumber("/Wrist/Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Wrist/Actual Angle", GetMeasurement().value());

    double fb;
    units::volt_t ff;
    units::volt_t v;

    switch (m_WristState)
    {
    case WristConstants::START:
    {

        frc::SmartDashboard::PutString("/Wrist/State", "Start");
        break;
    }
    case WristConstants::ZEROED:
    {
        if (!Zeroed.Get())
        {
            m_motor.Set(0.1);
        }
        else
        {
            m_encoder.SetPosition(90.0 / 360.0);
            m_WristState = WristConstants::HOLD;
            m_motor.Set(0);
        }
        frc::SmartDashboard::PutString("/Wrist/State", "Zeroed");
        break;
    }
    case WristConstants::MOVE:
    {
        m_controller.SetTolerance(WristConstants::kTolerancePos,
                                  WristConstants::kToleranceVel);
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(m_goal);

        frc::SmartDashboard::PutString("/Wrist/State", "AngleDown");

        if (m_controller.AtGoal())
        {
            m_WristState = WristConstants::HOLD;
        }
        else
        {
            fb = m_controller.Calculate(GetMeasurement());
            ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_WristSim.SetInputVoltage(v);
            }
            m_motor.SetVoltage(-v);

            frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_PID", fb);
            frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_FF", ff.value());
            frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_Volt", v.value());
            frc::SmartDashboard::PutString("/Wrist/WristState", "MOVING");
        }
        break;
    }
    case WristConstants::HOLD:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "Hold");
        double fb = m_controller.Calculate(GetMeasurement());
        units::volt_t ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        units::volt_t v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_WristSim.SetInputVoltage(v);
        }
        m_motor.SetVoltage(v);
        frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_PID", fb);
        frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_FF", ff.value());
        frc::SmartDashboard::PutNumber("/Wrist/Wrist_UO_Volt", v.value());
        frc::SmartDashboard::PutString("/Wrist/WristState", "MOVING");
        break;
    }
    default:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "default");
        break;
    }
    }
}

void Wrist::Zero()
{
    if (m_WristState != WristConstants::MOVE && m_WristState != WristConstants::HOLD)
    {
        m_goal = units::degree_t{GetMeasurement()};
        m_WristState = WristConstants::ZEROED;
    }
}

void Wrist::Enable()
{
    using State = frc::TrapezoidProfile<units::degrees>::State;
}

void Wrist::Disable()
{
    m_motor.StopMotor();
}