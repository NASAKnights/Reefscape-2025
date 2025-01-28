// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>

class POIGenerator
{
public:
  POIGenerator();
  void MakePOI();
  frc::Pose2d GetPOI(std::string poiKey);

private:
  nt::NetworkTableInstance networkTableInst;
  nt::DoubleArraySubscriber baseLinkSubscriber;
};
