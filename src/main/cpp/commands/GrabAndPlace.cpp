// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrabAndPlace.h"
#include <commands/DropAlgae.h>
#include <commands/MoveElevatorTo.h>
#include <commands/DropCoral.h>
#include <commands/Reset.h>
#include <commands/ChangeCoralAngle.h>
#include <commands/RunAlgaeIntake.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
GrabAndPlace::GrabAndPlace()
{

  MoveElevatorTo{}, ChangeCoralAngle{}, DropCoral{}, RunAlgaeIntake{}, Reset{};
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
