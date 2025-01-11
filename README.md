# 122 Reefscape 2025 Code Base

This is the code base for FRC 2025 Reefscape Game for Team 122 NASA Knights

## General Robot Specs

TBD: ???


## Changelog

###### Updated 1/11/25

- Will be updated to the new WPILib version 2025.2.1
- Removed the Util folder due to NKTrajectory and NKTrajectoryManager not working with the gradle build
- Updated Vendor libraries
- **Note:** Navx vendor library has changed to Studica
- Calibrate Function has been removed due to library changes
- How the Navx is instantiated is changed as well
**CTRE Changes**
- Pigeon has changed to GetYaw(), read specs as rotation direction has changed
**SwerveDriveModule**
- Optimize function has changed
**Pathplanner**
- Changes to how the SwerveAutoBuilder is instantiated
