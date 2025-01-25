// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

class POIGenerator
{
public:
  POIGenerator();
  void POIGenerator::MakePOI();

private:
  nt::NetworkTableInstance networkTableInst;
  nt::DoubleArraySubscriber baseLinkSubscriber;
};
