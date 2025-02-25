// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Reset.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Reset::Reset(Elevator *Elevator, Wrist *Wrist)
{
  // SetElevatorHeight{}, MoveWristToAngle{};
  // Add your commands here, e.g.
  AddCommands(SetElevatorHeight{m_elevator, 0}, MoveWristToAngle{m_wrist, 90.0});
  // TODO set height and angle to correct values
  //  AddCommands(FooCommand{}, BarCommand{});
}
