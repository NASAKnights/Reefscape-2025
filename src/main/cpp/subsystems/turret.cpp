// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/turret.h"

using State = frc::TrapezoidProfile<units::degrees>::State;

using degrees_per_second_squared_t = units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>;
Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD,
                       frc::TrapezoidProfile<units::degrees>::Constraints(TurretConstants::kArmVelLimit, TurretConstants::kArmAccelLimit), 5_ms),
                   m_motor(TurretConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless), m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV,
                                                                                                                            TurretConstants::kFFkA),
                   m_encoder{m_motor.GetEncoder()}
{
    m_controller.SetIZone(TurretConstants::kIZone);
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.encoder.PositionConversionFactor(360 / 81.0);
    config.SmartCurrentLimit(30, 0, 20000);

    m_motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    m_controller.SetTolerance(TurretConstants::kTolerancePos, TurretConstants::kToleranceVel);
    // Start m_arm in neutral position
    m_turretState = TurretConstants::DISABLED;

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Wrist/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Wrist/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Wrist/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorVoltage");
}

void Turret::GetTurretAngle()
{
}

void Turret::GetTurretSpeed()
{
}

void Turret::GetTargetAngle()
{
}
void Turret::Enable()
{
}

void Turret::Disable()
{
}

void Turret::SetTurretAngle(double targetangle)
{
}

void Turret::Periodic()
{
    printLog();
    double fb;
    units::volt_t ff;
    units::volt_t v;

    switch (m_turretState)
    {
    case TurretConstants::DISABLED:
    {
    }
    case TurretConstants::TRACKING:
    {
        GetTurretAngle();
        GetTurretSpeed();
        SetTurretAngle(double targetangle);
    }
    case TurretConstants::ZEROING:
    {
    }
    }
}

void Turret::SimulationPeriodic()
{
}

void Turret::printLog()
{
}