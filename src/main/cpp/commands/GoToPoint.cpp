#include "commands/GoToPoint.h"

using namespace pathplanner;

GoToPoint::GoToPoint(SwerveDrive *swerve, POIGenerator *poiGen) : m_swerve{swerve}, m_pathCommand{frc2::InstantCommand().ToPtr()}, m_poiGenerator{poiGen}
{
}

void GoToPoint::Initialize()
{
  using namespace pathplanner;
  using namespace frc;
  m_swerve->TurnVisionOn();
  Pose2d currentPose = m_swerve->GetPose();
  // Select Left or Right Branch
  frc::Transform2d offset = m_driverController.GetRawButton(7) ? frc::Transform2d(0.0_m, 0.35_m, frc::Rotation2d()) : frc::Transform2d(0.0_m, 0.0_m, frc::Rotation2d());

  // The rotation component in these poses represents the direction of travel
  Pose2d startPos = Pose2d(currentPose.Translation(), Rotation2d());
  Pose2d endPos = m_poiGenerator->GetClosestPOI().TransformBy(offset);

  auto transformedEndPos = endPos.TransformBy(Transform2d(0.35_m, 0_m, 0_rad));
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({startPos, endPos, transformedEndPos});
  // Paths must be used as shared pointers
  auto path = std::make_shared<PathPlannerPath>(
      waypoints,
      std::vector<RotationTarget>({RotationTarget(0.25, endPos.Rotation())}),
      std::vector<PointTowardsZone>(),
      std::vector<ConstraintsZone>(),
      std::vector<EventMarker>(),
      PathConstraints(1.5_mps, 2.0_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
      std::nullopt, // Ideal starting state can be nullopt for on-the-fly paths
      GoalEndState(0_mps, endPos.Rotation()),
      false);
  // Prevent this path from being flipped on the red alliance, since the given positions are already correct
  path->preventFlipping = true;

  m_pathCommand = frc2::CommandPtr(AutoBuilder::followPath(path).Unwrap());
  m_pathCommand.Schedule();
}

void GoToPoint::Execute()
{
}

void GoToPoint::End(bool interrupted)
{
  m_swerve->TurnVisionOff();
  m_pathCommand.Cancel();
}

bool GoToPoint::IsFinished()
{
  return m_pathCommand.IsScheduled();
}