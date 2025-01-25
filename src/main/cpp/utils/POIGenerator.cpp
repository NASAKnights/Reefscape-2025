// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/POIGenerator.h"

POIGenerator::POIGenerator() = default;

void POIGenerator::MakePOI()
{
    auto poiName = frc::SmartDashboard::GetString("POIName", "");
    auto baseLinkPose = baseLinkSubscriber.GetAtomic();

    frc::SmartDashboard::PutNumberArray(poiName, baseLinkPose.value);
}
