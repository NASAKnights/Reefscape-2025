// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RaiseElevator.h"

RaiseElevator::RaiseElevator() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RaiseElevator::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RaiseElevator::Execute() {}

// Called once the command ends or is interrupted.
void RaiseElevator::End(bool interrupted) {}

// Returns true when the command should end.
bool RaiseElevator::IsFinished() {
  return false;
}
