package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefChosenOrientation;
import frc.robot.FieldConstants.ReefFacesBlue;
import frc.robot.FieldConstants.ReefFacesRed;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToCommands {
  private DriveToCommands() {
    // Utility class - no instances
  }

  /**
   * Drive to a specific pose on the field.
   *
   * @param drive The drivetrain subsystem.
   * @param poseSupplier A supplier for the target pose.
   * @return A command to drive to the specified pose.
   */
  public static Command createDriveToPose(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier) {
    return new DriveToPose(
        drive, poseSupplier, xJoystickSupplier, yJoystickSupplier, rotationJoystickSupplier);
  }

  public static Command driveToPole(
      Drive drive,
      boolean isLeftPole,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double distanceThresholdMeters) {
    return Commands.runOnce(
        () -> {
          // Get the closest reef face selection
          AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();

          // Check if a valid face is found
          if (selection == null || selection.getAcceptedFaceId() == null) {
            System.out.println("No valid reef face found. Cannot drive to pole.");
            return;
          }

          // Check if the accepted face is within the distance threshold
          double closestDistance = selection.getAcceptedDistance();
          if (closestDistance > distanceThresholdMeters) {
            System.out.println(
                "Closest reef face is too far: "
                    + closestDistance
                    + " meters (threshold: "
                    + distanceThresholdMeters
                    + " meters).");
            return;
          }

          // Calculate the target pole pose
          Pose2d targetPose = calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);
          if (targetPose != null) {
            System.out.println("Target pose calculated: " + targetPose);
            createDriveToPose(
                    drive,
                    () -> targetPose,
                    xJoystickSupplier,
                    yJoystickSupplier,
                    rotationJoystickSupplier)
                .schedule();
          } else {
            System.out.println("Failed to calculate target pose for pole.");
          }
        },
        drive);
  }

  public static Pose2d calculatePolePose(Drive drive, int faceId, boolean isLeftPole) {
    Translation2d poleTranslation;

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      ReefFacesBlue blueFace = ReefFacesBlue.fromId(faceId);
      if (blueFace == null) {
        System.out.println("Invalid face ID for Blue alliance.");
        return null;
      }

      // Select left/right and front/back
      ReefChosenOrientation chosen =
          AlignmentUtils.pickClosestOrientationForReef(drive.getPose(), faceId);
      poleTranslation =
          isLeftPole
              ? chosen.orientationType() == FieldConstants.ReefOrientationType.FRONT
                  ? blueFace.getLeftPole().getFrontTranslation()
                  : blueFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == FieldConstants.ReefOrientationType.FRONT
                  ? blueFace.getRightPole().getFrontTranslation()
                  : blueFace.getRightPole().getBackTranslation();

      // Return the pose with proper orientation
      return new Pose2d(poleTranslation, chosen.rotation2D());

    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      ReefFacesRed redFace = ReefFacesRed.fromId(faceId);
      if (redFace == null) {
        System.out.println("Invalid face ID for Red alliance.");
        return null;
      }

      // Select left/right and front/back
      ReefChosenOrientation chosen =
          AlignmentUtils.pickClosestOrientationForReef(drive.getPose(), faceId);
      poleTranslation =
          isLeftPole
              ? chosen.orientationType() == FieldConstants.ReefOrientationType.FRONT
                  ? redFace.getLeftPole().getFrontTranslation()
                  : redFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == FieldConstants.ReefOrientationType.FRONT
                  ? redFace.getRightPole().getFrontTranslation()
                  : redFace.getRightPole().getBackTranslation();

      // Return the pose with proper orientation
      return new Pose2d(poleTranslation, chosen.rotation2D());

    } else {
      System.out.println("Unknown alliance. Cannot calculate target pose.");
      return null;
    }
  }

  /**
   * Drive to the closest coral station face (based on which station AprilTag is closest).
   *
   * @param drive The drivetrain subsystem.
   * @param xJoystickSupplier The forward/back joystick input.
   * @param yJoystickSupplier The left/right joystick input.
   * @param rotationJoystickSupplier The rotation joystick input.
   * @param distanceThresholdMeters If the station is farther than this threshold, do nothing.
   */
  public static Command driveToCoralStation(
      Drive drive,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double distanceThresholdMeters) {

    return Commands.runOnce(
        () -> {
          // 1) Ask AlignmentUtils which station face is closest:
          AlignmentUtils.CoralStationSelection selection =
              AlignmentUtils.findClosestCoralStation(drive.getPose());

          if (selection == null || selection.getAcceptedStationId() == null) {
            System.out.println("No valid coral station face found. Cannot drive to station.");
            return;
          }

          // 2) Check distance threshold
          double closestDistance = selection.getAcceptedDistance();
          if (closestDistance > distanceThresholdMeters) {
            System.out.println(
                "Closest coral station is too far: "
                    + closestDistance
                    + " meters (threshold: "
                    + distanceThresholdMeters
                    + ")");
            return;
          }

          // 3) Calculate target pose
          Pose2d targetPose = calculateCoralStationPose(drive, selection.getAcceptedStationId());
          if (targetPose != null) {
            System.out.println("Target coral station pose: " + targetPose);

            // 4) Schedule the actual "drive to pose" command
            createDriveToPose(
                    drive,
                    () -> targetPose,
                    xJoystickSupplier,
                    yJoystickSupplier,
                    rotationJoystickSupplier)
                .schedule();
          } else {
            System.out.println("Failed to calculate coral station pose.");
          }
        },
        drive);
  }

  /**
   * Uses the station ID to look up the correct AprilTag translation and orientation, then returns
   * the final Pose2d (position + heading) for the robot.
   */
  private static Pose2d calculateCoralStationPose(Drive drive, int stationId) {
    // 1) Grab alliance
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();

    // 2) Based on alliance, get the translation for the station
    //    (We've already done the "closest station" logic. This method just
    //     converts station ID to a final "pose to aim for.")
    switch (alliance) {
      case Blue -> {
        Translation2d stationTranslation = new Translation2d(.94, 1.1);

        // Pick an orientation
        var chosenOrientation =
            AlignmentUtils.pickClosestOrientationForStation(drive.getPose(), stationId);
        return new Pose2d(stationTranslation, chosenOrientation.rotation2D());
      }
      case Red -> {
        var stationTranslation = FieldConstants.RED_CORALSTATION_APRIL_TAGS.get(stationId);
        if (stationTranslation == null) {
          System.out.println("No known coral station for Red face ID: " + stationId);
          return null;
        }

        var chosenOrientation =
            AlignmentUtils.pickClosestOrientationForStation(drive.getPose(), stationId);
        return new Pose2d(stationTranslation, chosenOrientation.rotation2D());
      }
      default -> {
        System.out.println("Unknown alliance. Cannot calculate coral station pose.");
        return null;
      }
    }
  }
}
