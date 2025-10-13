// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD,
                       frc::TrapezoidProfile<units::degrees>::Constraints(TurretConstants::kTurretVelLimit, TurretConstants::kTurretAccelLimit), 5_ms),

                   m_motor(TurretConstants::kAngleMotorId), m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV,
                                                                          TurretConstants::kFFkA),

                   m_TurretSim(TurretConstants::kSimMotor, TurretConstants::kGearRatio, TurretConstants::kmoi,
                               TurretConstants::kTurretRadius, TurretConstants::kminAngle, TurretConstants::kmaxAngle,
                               TurretConstants::kGravity, TurretConstants::kTurretStartAngle, TurretConstants::kSimNoise)
{
    m_controller.SetIZone(TurretConstants::kIZone);

    m_controller.SetTolerance(TurretConstants::kTolerancePos, TurretConstants::kToleranceVel);
    // Start m_Turret in neutral position
    m_TurretState = TurretConstants::DISABLED;
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
    // const frc::DCMotor, const double, const units::moment_of_inertia::kilogram_square_meter_t, const units::length::meter_t,
    // const units::angle::radian_t, const units::angle::radian_t, const bool, const units::angle::radian_t, const std::array<double, 1U>)
}

void Turret::SimulationPeriodic()
{
    m_TurretSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_TurretSim.GetCurrentDraw().value());
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_TurretSim.GetAngle();
    }

    return units::turn_t{(m_motor.GetPosition().GetValue() / TurretConstants::kGearRatio)};
}

void Turret::SetAngle(double TurretAngleGoal)
{
    // m_TurretState = TurretConstants::MOVE;
    if (TurretAngleGoal != m_goal.value())
    {
        if ((TurretAngleGoal <= double(TurretConstants::kmaxAngle.convert<units::angle::degree>())) && (TurretAngleGoal >= double(TurretConstants::kminAngle.convert<units::degree>())))
            m_TurretState = TurretConstants::START;
        m_goal = units::angle::degree_t(TurretAngleGoal);
    }
    // m_controller.Reset(GetMeasurement());
    // m_controller.SetGoal(m_goal);
    frc::SmartDashboard::PutNumber("/Turret/m_goal", double(m_goal));
}

void Turret::Periodic()
{

    printLog();
    double fb;
    units::volt_t ff;
    units::volt_t v;

    switch (m_TurretState)
    {
    case TurretConstants::START:
    {
        frc::SmartDashboard::PutString("/Turret/State", "START");
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(m_goal);
        m_TurretState = TurretConstants::MOVE;
    }
    case TurretConstants::MOVE:
    {

        frc::SmartDashboard::PutString("/Turret/State", "MOVE");
        if (m_controller.AtGoal())
        {
            m_TurretState = TurretConstants::HOLD;
        }
        else
        {
            fb = m_controller.Calculate(GetMeasurement());
            ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_TurretSim.SetInputVoltage(v);
            }
            m_motor.SetVoltage(v);
        }
        break;
    }
    case TurretConstants::HOLD:
    {
        frc::SmartDashboard::PutString("/Turret/State", "HOLD");
        double fb = m_controller.Calculate(GetMeasurement());
        units::volt_t ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        units::volt_t v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_TurretSim.SetInputVoltage(v);
        }
        m_motor.SetVoltage(v);
        break;
    }
    case TurretConstants::DISABLED:
    {
        frc::SmartDashboard::PutString("/Turret/State", "DISABLED");
        break;
    }
    case TurretConstants::TRACKING:
    {
        // do math stuff perchance

        // Minimize
        // theta = arccos((vector(turret2goal) * <1,0>)/(||vector(turret2goal|| * ||<1,0>||)) + arccos((trace(world2turret_rotation_matrix) - 1)/2)

        // Measurement: angle between world y axis and vector(turret2goal)
        // arccos((vector(turret2goal) * <1,0>)/(||vector(turret2goal|| * ||<1,0>||)) = angle between world y axis and vector(turret2goal)

        // Control Var/ Control angle:
        // arccos((trace(world2turret_rotation_matrix) - 1)/2) = angle between turret y axis and world y axis

        // world2robot comes in 2D pose: SwerveDrive.Getpose() ?
        // world2goal comes from camera
        // robot2turret comes from 3D pose and encoder
        // world2turret rotation matrix: how get?

        // grab world2robot, world2goal, robot2turret transforms
        // get world2turret transform from world2robot * robot2turret
        // get turret2goal transform from (world2turret)^-1 * world2goal
        // get turret2goal vector from turret2goal transform
        // grab rotation matrix from world2turret transform
        // Calc control angle to make theta 0
        // set goal angle to control Var angle needed to make theta zero
    }
    default:
    {
        frc::SmartDashboard::PutString("/Turret/State", "default");
        break;
    }
    }
}

TurretConstants::TurretState Turret::GetState()
{
    return m_TurretState;
}

void Turret::printLog()
{
    frc::SmartDashboard::PutNumber("/Turret/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Turret/Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Turret/setpoint",
                                   m_controller.GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_TurretState);
    // m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    // m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());zz
}

void Turret::Disable()
{
    m_motor.StopMotor();
}

void Turret::HoldPosition()
{
    // if (m_TurretState != TurretConstants::TurretState::HOLD)
    {
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(GetMeasurement());
        m_goal = GetMeasurement();
        m_TurretState = TurretConstants::TurretState::HOLD;
    }
}