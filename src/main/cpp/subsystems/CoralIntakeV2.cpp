// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralIntakeV2.h"

CoralIntakeV2::CoralIntakeV2() : coralIntakeMotor{4, rev::spark::SparkLowLevel::MotorType::kBrushless} {};

// This method will be called once per scheduler run
void CoralIntakeV2::Periodic()
{
    hasCoral();
}

void CoralIntakeV2::SetConfig()
{
    if (!Configure)
    {
        // ctre::phoenix::motorcontrol::can::TalonSRXConfiguration config;
        // config.peakCurrentLimit = 40;
        // config.peakCurrentDuration = 1500;
        // config.continuousCurrentLimit = 18;
        // coralIntakeMotor.ConfigAllSettings(config);
        // coralIntakeMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

        rev::spark::SparkBaseConfig config;
        config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
        config.SmartCurrentLimit(20);
        coralIntakeMotor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

        Configure = true;
    }
}

void CoralIntakeV2::Intake(double Speed)
{
    coralIntakeMotor.Set(Speed);
}

void CoralIntakeV2::Outtake(double Speed)
{
    coralIntakeMotor.Set(Speed);
}

void CoralIntakeV2::stopMotors()
{
    coralIntakeMotor.Set(0.0);
}

bool CoralIntakeV2::hasCoral()
{

    frc::SmartDashboard::PutNumber("HAS CORAL", coralIntakeMotor.GetReverseLimitSwitch().Get());
    timerV2.Start();
    return coralIntakeMotor.GetReverseLimitSwitch().Get();
}