// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DeployClimb.h"

DeployClimb::DeployClimb(Climber *_climber) : m_climber{_climber}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_climber);
}

// Called when the command is initially scheduled.
void DeployClimb::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DeployClimb::Execute()
{
  m_climber->Deploy();
}

// Called once the command ends or is interrupted.
void DeployClimb::End(bool interrupted)
{
  m_climber->Stop();
}

// Returns true when the command should end.
bool DeployClimb::IsFinished()
{
  return false;
}
