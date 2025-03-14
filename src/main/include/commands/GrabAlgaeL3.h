// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "commands/SetElevatorHeight.h"
#include "commands/Reset.h"
#include "commands/RunAlgaeIntake.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class GrabAlgaeL3
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 GrabAlgaeL3>
{
public:
  GrabAlgaeL3(IntakeAlgae *intakeAlgae);

  IntakeAlgae *m_intakeAlgae;

private:
};
