// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/Wrist.h"
#include "subsystems/Elevator.h"
#include "subsystems/IntakeCoral.h"
#include "commands/Reset.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class PlaceL2
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 PlaceL2>
{
public:
  PlaceL2(Wrist *wrist, Elevator *elevator);
};
