// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunCoralOuttake.h"

RunCoralOuttake::RunCoralOuttake()
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunCoralOuttake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunCoralOuttake::Execute()
{
  intakeCoral.Outtake(0.5);
}

// Called once the command ends or is interrupted.
void RunCoralOuttake::End(bool interrupted) {}

// Returns true when the command should end.
bool RunCoralOuttake::IsFinished()
{
  return false;
}
