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
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

  public static Pose2d calculateL1Pose(Drive drive, int faceId, boolean isLeft) {
    Translation2d midpointTranslation;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (!alliance.isPresent()) {
      Logger.recordOutput(
          "Alignment/calculateL1Pose", "Unknown alliance. Cannot calculate L1 target pose.");
      return null;
    }

    Logger.recordOutput("Alignment/calculateL1Pose/Alliance", alliance.get().toString());

    if (alliance.get() == DriverStation.Alliance.Blue) {
      ReefFacesBlue blueFace = ReefFacesBlue.fromId(faceId);
      if (blueFace == null) {
        Logger.recordOutput(
            "Alignment/calculateL1Pose", "Invalid face ID for Blue alliance: " + faceId);
        return null;
      }
      Logger.recordOutput("Alignment/calculateL1Pose/BlueFace", blueFace.toString());

      // Compute the midpoint between left and right poles
      Translation2d leftTranslation = blueFace.getLeftPole().getBranchTranslation();
      Translation2d rightTranslation = blueFace.getRightPole().getBranchTranslation();
      Logger.recordOutput("Alignment/calculateL1Pose/LeftTranslation", leftTranslation);
      Logger.recordOutput("Alignment/calculateL1Pose/RightTranslation", rightTranslation);

      midpointTranslation = leftTranslation.plus(rightTranslation).div(2.0);
      Logger.recordOutput("Alignment/calculateL1Pose/MidpointTranslation", midpointTranslation);

    } else if (alliance.get() == DriverStation.Alliance.Red) {
      ReefFacesRed redFace = ReefFacesRed.fromId(faceId);
      if (redFace == null) {
        Logger.recordOutput(
            "Alignment/calculateL1Pose", "Invalid face ID for Red alliance: " + faceId);
        return null;
      }
      Logger.recordOutput("Alignment/calculateL1Pose/RedFace", redFace.toString());

      // Compute the midpoint between left and right poles
      Translation2d leftTranslation = redFace.getLeftPole().getBranchTranslation();
      Translation2d rightTranslation = redFace.getRightPole().getBranchTranslation();
      Logger.recordOutput("Alignment/calculateL1Pose/LeftTranslation", leftTranslation);
      Logger.recordOutput("Alignment/calculateL1Pose/RightTranslation", rightTranslation);

      midpointTranslation = leftTranslation.plus(rightTranslation).div(2.0);
      Logger.recordOutput("Alignment/calculateL1Pose/MidpointTranslation", midpointTranslation);

    } else {
      Logger.recordOutput(
          "Alignment/calculateL1Pose", "Unknown alliance. Cannot calculate L1 target pose.");
      return null;
    }

    // Determine closest reef orientation and apply a 180-degree rotation
    ReefChosenOrientation chosen =
        AlignmentUtils.pickClosestOrientationForReef(drive.getPose(), faceId);
    if (chosen == null) {
      Logger.recordOutput(
          "Alignment/calculateL1Pose",
          "Failed to determine reef orientation for face ID: " + faceId);
      return null;
    }
    Logger.recordOutput("Alignment/calculateL1Pose/ChosenOrientation", chosen.toString());

    Rotation2d flippedRotation = chosen.rotation2D().plus(Rotation2d.fromDegrees(180));
    Logger.recordOutput("Alignment/calculateL1Pose/FlippedRotation", flippedRotation);

    // Offset distance to move the robot *right* in the flipped orientation
    double rightOffset = 0.38; // Adjust as needed based on actual misalignment
    Logger.recordOutput("Alignment/calculateL1Pose/RightOffset", rightOffset);

    // Shift right in the robot's new coordinate frame (perpendicular to heading)
    // this is just to account for the robot being flipped 180
    Translation2d rightShift = new Translation2d(0.0, rightOffset).rotateBy(flippedRotation);
    Logger.recordOutput("Alignment/calculateL1Pose/RightShift", rightShift);

    Translation2d adjustedMidpoint = midpointTranslation.plus(rightShift);
    Logger.recordOutput("Alignment/calculateL1Pose/AdjustedMidpoint", adjustedMidpoint);

    // Offset from center left/right depending on button pushed
    double offsetDistance = 0.03;

    // If isLeft is true, use a negative offset (to shift left); otherwise, use positive (to shift
    // right)
    double offset = isLeft ? -offsetDistance : offsetDistance;

    // Shift in the robot's new coordinate frame (perpendicular to heading)
    Translation2d offsetShift = new Translation2d(0.0, offset).rotateBy(flippedRotation);
    Logger.recordOutput("Alignment/calculateL1Pose/OffsetShift", offsetShift);

    Translation2d reAdjustedMidpoint = adjustedMidpoint.plus(offsetShift);
    Logger.recordOutput("Alignment/calculateL1Pose/ReAdjustedMidpoint", reAdjustedMidpoint);

    // Return the final pose with adjusted translation and flipped rotation
    Pose2d finalPose = new Pose2d(reAdjustedMidpoint, flippedRotation);
    Logger.recordOutput("Alignment/calculateL1Pose/FinalPose", finalPose);

    return finalPose;
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

    // If we are in L1 scoring we have to return a different standoff.
    if (RobotStatus.getTargetPreset() == CoralSystemPresets.L1_SCORE) {
      Translation2d offset =
          new Translation2d(standoffDistanceMeters, 0.0).rotateBy(targetPose.getRotation());
      return new Pose2d(
          targetPose.getX() + offset.getX(),
          targetPose.getY() + offset.getY(),
          targetPose.getRotation());
    }

    Translation2d offset =
        new Translation2d(-standoffDistanceMeters, 0.0).rotateBy(targetPose.getRotation());
    return new Pose2d(
        targetPose.getX() + offset.getX(),
        targetPose.getY() + offset.getY(),
        targetPose.getRotation());
  }
}
