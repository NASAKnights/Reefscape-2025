#include "commands/GoToPoint.h"

using namespace pathplanner;

GoToPoint::GoToPoint(std::string poiName, SwerveDrive *swerve) : m_swerve{swerve}, m_POIName{poiName}, m_pathCommand{frc2::InstantCommand().ToPtr()}
{
}

void GoToPoint::Initialize()
{
  // TODO Get the Pose2d from the shuffleboard of the POI
  auto pose = POIGenerator::GetPOI(m_POIName);
  // Create a vector of waypoints from poses. Each pose represents one waypoint.
  // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
  std::vector<frc::Pose2d> poses{m_swerve->GetPose(), pose};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);
  PathConstraints constraints(2.0_mps, 2.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq); // The constraints for this path.
  // PathConstraints constraints = PathConstraints::unlimitedConstraints(12_V); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

  AutoBuilder::pathfindToPose(pose, constraints).Unwrap();
  // Create the path using the waypoints created above
  // We make a shared pointer here since the path following commands require a shared pointer
  auto path = std::make_shared<PathPlannerPath>(
      waypoints,
      constraints,
      std::nullopt,                                   // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
      GoalEndState(0.0_mps, frc::Rotation2d(-90_deg)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path->preventFlipping = true;
  // Add the path-following command to the command group
  m_pathCommand = pathplanner::AutoBuilder::followPath(path);
  frc::SmartDashboard::PutNumber("IM AT", 6);
  m_pathCommand.Schedule();
}

void GoToPoint::Execute()
{
}

void GoToPoint::End(bool interrupted)
{
  m_pathCommand.Cancel();
}

bool GoToPoint::IsFinished()
{
  return m_pathCommand.IsScheduled();
}