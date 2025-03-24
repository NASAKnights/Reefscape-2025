// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeCoral.h"

IntakeCoral::IntakeCoral() : coralIntakeMotor{4, rev::spark::SparkLowLevel::MotorType::kBrushless} {};

// This method will be called once per scheduler run
void IntakeCoral::Periodic() {}

void IntakeCoral::SetConfig()
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

void IntakeCoral::Intake(double Speed)
{
    coralIntakeMotor.Set(Speed);
}

void IntakeCoral::Outtake(double Speed)
{
    coralIntakeMotor.Set(Speed);
}

void IntakeCoral::stopMotors()
{
    coralIntakeMotor.Set(0.0);
}

bool IntakeCoral::hasCoral()
{
    return coralIntakeMotor.GetReverseLimitSwitch().Get();
}