// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceL3.h"
#include <commands/SetElevatorHeight.h>
#include <commands/DropCoral.h>
#include <commands/Reset.h>
#include <commands/ChangeIntakeAngle.h>
#include <frc2/command/ParallelCommandGroup.h>
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceL3::PlaceL3()
{
  // AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight(), ChangeIntakeAngle()), DropCoral(), Reset());
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
