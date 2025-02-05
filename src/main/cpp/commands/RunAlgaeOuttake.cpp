// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAlgaeOuttake.h"

RunAlgaeOuttake::RunAlgaeOuttake()
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunAlgaeOuttake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunAlgaeOuttake::Execute()
{
  intakeAlgae.Outtake(0.5);
}

// Called once the command ends or is interrupted.
void RunAlgaeOuttake::End(bool interrupted) {}

// Returns true when the command should end.
bool RunAlgaeOuttake::IsFinished()
{
  return false;
}
