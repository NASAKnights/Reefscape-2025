// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/POIGenerator.h"

POIGenerator::POIGenerator() : networkTableInst(nt::NetworkTableInstance::GetDefault())
{
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");

    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.01, .sendAll = true});
    auto inst = nt::NetworkTableInstance::GetDefault();
    frc::SmartDashboard::PutData("ClosestPOI", &closestPOI);
    std::vector<nt::Topic> topics = inst.GetTopics("/SmartDashboard/POI");
    std::string prefix = "/SmartDashboard/";
    for (auto topic : topics)
    {
        std::string name = topic.GetName();
        // Remove "/SmartDashboard" from the name
        if (name.rfind(prefix, 0) == 0)
        {                                        // Check if it starts with the prefix
            name = name.substr(prefix.length()); // Remove "/SmartDashboard"
        }
        auto pose = GetPOI(name);
        if (pose != frc::Pose2d(0_m, 0_m, 0_rad))
        {
            poses.push_back(pose);
        }
    }
}

void POIGenerator::MakePOI()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    auto baseLinkPose = baseLinkSubscriber.GetAtomic();

    frc::SmartDashboard::PutNumberArray(poiName, baseLinkPose.value);
    frc::SmartDashboard::SetPersistent(poiName);
}

frc::Pose2d POIGenerator::GetPOI(std::string poiKey)
{
    std::vector<double> numberArray = frc::SmartDashboard::GetNumberArray(poiKey, {});
    return POIGenerator::DoubleArrayToPose2d(numberArray);
}

void POIGenerator::RemovePOI()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");

    frc::SmartDashboard::ClearPersistent(poiName);
}

frc::Pose2d POIGenerator::GetClosestPOI()
{
    std::vector<double> baseLinkPose = baseLinkSubscriber.GetAtomic().value;
    auto baseLink = DoubleArrayToPose2d(baseLinkPose);
    auto pose = baseLink.Nearest(poses);
    closestPOI.SetRobotPose(pose);
    frc::SmartDashboard::PutNumber("TARGET POI X", pose.X().value());
    frc::SmartDashboard::PutNumber("TARGET POI Y", pose.Y().value());
    return pose;
}