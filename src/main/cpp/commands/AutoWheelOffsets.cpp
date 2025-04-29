// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoWheelOffsets.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoWheelOffsets::AutoWheelOffsets()
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoWheelOffsets::Initialize()
{

  double WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(1) + "/" + " Reported Angle", 0);

  frc::SmartDashboard::PutNumber("FrontLeftDegree", WheelOffsets);
  // frc::SmartDashboard::PutNumber ("FrontLeftDegree", "Module " + std::to_string(m_id) + "/" + " Reported Angle");
  frc::SmartDashboard::SetPersistent("FrontLeftDegree");
}

// Called repeatedly when this Command is scheduled to run
void AutoWheelOffsets::Execute() {}

// Called once the command ends or is interrupted.
void AutoWheelOffsets::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoWheelOffsets::IsFinished()
{
  return true;
}
