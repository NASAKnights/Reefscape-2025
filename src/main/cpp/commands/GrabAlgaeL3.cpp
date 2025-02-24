// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrabAlgaeL3.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
GrabAlgaeL3::GrabAlgaeL3(IntakeAlgae *_intakeAlgae) : m_intakeAlgae{_intakeAlgae}
{
  // AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight(), ChangeIntakeAngle()), RunAlgaeIntake(m_intakeAlgae), Reset());
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
