package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefChosenOrientation;
import frc.robot.FieldConstants.ReefFacesBlue;
import frc.robot.FieldConstants.ReefFacesRed;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToCommands {
  private DriveToCommands() {
    // Utility class - no instances
  }

  public static Command driveToPole(
      Drive drive, boolean isLeftPole, double distanceThresholdMeters) {
    Supplier<Pose2d> polePoseSupplier =
        createPolePoseSupplier(drive, isLeftPole, distanceThresholdMeters);
    return new DriveToPose(drive, polePoseSupplier).unless(() -> polePoseSupplier.get() == null);
  }

  public static Supplier<Pose2d> createPolePoseSupplier(
      Drive drive, boolean isLeftPole, double distanceThresholdMeters) {
    return () -> {
      if (!RobotStatus.haveCoral()) {
        System.out.println("No coral, skipping pose calc.");
        return null;
      }

      var selection = drive.getReefFaceSelection();
      if (selection == null || selection.getAcceptedFaceId() == null) {
        System.out.println("No valid face ID.");
        return null;
      }

      if (selection.getAcceptedDistance() > distanceThresholdMeters) {
        System.out.println("Too far from reef: " + selection.getAcceptedDistance());
        return null;
      }

      return calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);
    };
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
              ? blueFace.getLeftPole().getBranchTranslation()
              : blueFace.getRightPole().getBranchTranslation();

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
              ? redFace.getLeftPole().getBranchTranslation()
              : redFace.getRightPole().getBranchTranslation();

      // Return the pose with proper orientation
      return new Pose2d(poleTranslation, chosen.rotation2D());

    } else {
      System.out.println("Unknown alliance. Cannot calculate target pose.");
      return null;
    }
  }

  public static Pose2d calculateL1Pose(Drive drive, int faceId) {
    Translation2d midpointTranslation;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (!alliance.isPresent()) {
      System.out.println("Unknown alliance. Cannot calculate L1 target pose.");
      return null;
    }

    if (alliance.get() == DriverStation.Alliance.Blue) {
      ReefFacesBlue blueFace = ReefFacesBlue.fromId(faceId);
      if (blueFace == null) {
        System.out.println("Invalid face ID for Blue alliance: " + faceId);
        return null;
      }

      // Compute the midpoint between left and right poles
      midpointTranslation =
          blueFace
              .getLeftPole()
              .getBranchTranslation()
              .plus(blueFace.getRightPole().getBranchTranslation())
              .div(2.0); // Average the two translations to get the midpoint

    } else if (alliance.get() == DriverStation.Alliance.Red) {
      ReefFacesRed redFace = ReefFacesRed.fromId(faceId);
      if (redFace == null) {
        System.out.println("Invalid face ID for Red alliance: " + faceId);
        return null;
      }

      // Compute the midpoint between left and right poles
      midpointTranslation =
          redFace
              .getLeftPole()
              .getBranchTranslation()
              .plus(redFace.getRightPole().getBranchTranslation())
              .div(2.0);

    } else {
      System.out.println("Unknown alliance. Cannot calculate L1 target pose.");
      return null;
    }

    // Determine closest reef orientation and apply a 180-degree rotation
    ReefChosenOrientation chosen =
        AlignmentUtils.pickClosestOrientationForReef(drive.getPose(), faceId);
    if (chosen == null) {
      System.out.println("Failed to determine reef orientation for face ID: " + faceId);
      return null;
    }

    Rotation2d flippedRotation = chosen.rotation2D().plus(Rotation2d.fromDegrees(180));

    // Offset distance to move the robot *right* in the flipped orientation
    double rightOffset = 0.38; // Adjust as needed based on actual misalignment

    // Shift right in the robot's new coordinate frame (perpendicular to heading)
    Translation2d adjustedMidpoint =
        midpointTranslation.plus(
            new Translation2d(0.0, rightOffset)
                .rotateBy(flippedRotation) // Moves "right" relative to the flipped heading
            );

    // Return the final pose with adjusted translation and flipped rotation
    return new Pose2d(adjustedMidpoint, flippedRotation);
  }

  public static Command driveToCoralStation(
      Drive drive,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double distanceThresholdMeters) {

    Supplier<Pose2d> poseSupplier =
        () -> {
          AlignmentUtils.CoralStationSelection selection =
              AlignmentUtils.findClosestCoralStation(drive.getPose());

          if (selection == null || selection.getAcceptedStationId() == null) {
            System.out.println("No valid coral station face found. Cannot drive to station.");
            return null;
          }

          double closestDistance = selection.getAcceptedDistance();
          if (closestDistance > distanceThresholdMeters) {
            System.out.println(
                "Closest coral station is too far: "
                    + closestDistance
                    + " meters (threshold: "
                    + distanceThresholdMeters
                    + ")");
            return null;
          }

          Pose2d targetPose = calculateCoralStationPose(drive, selection.getAcceptedStationId());
          if (targetPose == null) {
            System.out.println("Failed to calculate coral station pose.");
          }
          return targetPose;
        };

    return new DriveToPoseJoystickCancel(
            drive, poseSupplier, xJoystickSupplier, yJoystickSupplier, rotationJoystickSupplier)
        .unless(() -> poseSupplier.get() == null);
  }

  /**
   * Uses the station ID to look up the correct AprilTag translation and orientation, then returns
   * the final Pose2d (position + heading) for the robot.
   */
  private static Pose2d calculateCoralStationPose(Drive drive, int stationId) {
    // 1) Grab alliance
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();

    // 2) Based on alliance, get the translation for the station
    // (We've already done the "closest station" logic. This method just
    // converts station ID to a final "pose to aim for.")
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

  /** Calculate standoff pose N meters away from the reef pose */
  public static Pose2d calculateStandoffPose(Pose2d targetPose, double standoffDistanceMeters) {
    Translation2d offset =
        new Translation2d(-standoffDistanceMeters, 0.0).rotateBy(targetPose.getRotation());
    return new Pose2d(
        targetPose.getX() + offset.getX(),
        targetPose.getY() + offset.getY(),
        targetPose.getRotation());
  }
}
