// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This Changes The Coral Intake Angle, It Would Be Used In Place L1-4, GrabAndPlace, GrabCoral, Etc

#include "commands/ChangeCoralAngle.h"

ChangeCoralAngle::ChangeCoralAngle()
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ChangeCoralAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChangeCoralAngle::Execute() {}

// Called once the command ends or is interrupted.
void ChangeCoralAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeCoralAngle::IsFinished()
{
  return false;
}
