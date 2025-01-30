// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbCage.h"

ClimbCage::ClimbCage() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ClimbCage::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimbCage::Execute() {}

// Called once the command ends or is interrupted.
void ClimbCage::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbCage::IsFinished() {
  return false;
}
