// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrabCoral.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
GrabCoral::GrabCoral(IntakeCoral *_intakeCoral) : m_intakeCoral{_intakeCoral}
{
  AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight(), ChangeIntakeAngle()), RunCoralIntake(m_intakeCoral), Reset());
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
