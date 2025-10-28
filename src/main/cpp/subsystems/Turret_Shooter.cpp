// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret_Shooter.h"

Turret_Shooter::Turret_Shooter()
{

    m_followerShooterMotor.Configure(followerShooterMotorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_mainShooterMotor.Configure(mainShooterMotorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    mainShooterMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    followerShooterMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    mainShooterMotorConfig.SmartCurrentLimit(30);
    followerShooterMotorConfig.SmartCurrentLimit(30);
}

void Turret_Shooter::SetSpeed(double speed)
{

    .setrefrence
}

// This method will be called once per scheduler run
void Turret_Shooter::Periodic() {}
