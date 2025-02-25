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
#include "commands/ScoreCoral.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class PlaceL3
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 PlaceL3>
{
public:
  PlaceL3(Wrist *wrist, Elevator *Elevator);

  Wrist *m_wrist;
  Elevator *m_elevator;
  IntakeCoral *m_intakeCoral;
};
