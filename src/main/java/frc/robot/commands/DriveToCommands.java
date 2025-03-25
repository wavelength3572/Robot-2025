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
import java.util.Optional;
import java.util.function.BooleanSupplier;
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
    return new DriveToPoseNoJoystick(drive, poseSupplier);
  }

  /**
   * Drive to a specific pose on the field.
   *
   * @param drive The drivetrain subsystem.
   * @param poseSupplier A supplier for the target pose.
   * @return A command to drive to the specified pose.
   */
  public static Command createDriveToPoseScaled(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double speedScalar) {
    return new DriveToPose(
        drive,
        poseSupplier,
        xJoystickSupplier,
        yJoystickSupplier,
        rotationJoystickSupplier,
        speedScalar);
  }

  public static Command driveToPole(
      Drive drive,
      boolean isLeftPole,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double distanceThresholdMeters,
      BooleanSupplier haveCoralSupplier) {

    Supplier<Pose2d> dynamicPoseSupplier =
        createDynamicPolePoseSupplier(
            drive, isLeftPole, distanceThresholdMeters, haveCoralSupplier);

    return createDriveToPose(
            drive,
            dynamicPoseSupplier,
            xJoystickSupplier,
            yJoystickSupplier,
            rotationJoystickSupplier)
        .unless(() -> dynamicPoseSupplier.get() == null);
  }

  public static Supplier<Pose2d> createDynamicPolePoseSupplier(
      Drive drive,
      boolean isLeftPole,
      double distanceThresholdMeters,
      BooleanSupplier haveCoralSupplier) {

    return () -> {
      if (!haveCoralSupplier.getAsBoolean()) {
        System.out.println("No coral in the robot. Aborting drive to pole.");
        return null;
      }

      AlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
      if (selection == null || selection.getAcceptedFaceId() == null) {
        System.out.println("No valid reef face found.");
        return null;
      }

      double closestDistance = selection.getAcceptedDistance();
      if (closestDistance > distanceThresholdMeters) {
        System.out.println("Reef face too far: " + closestDistance + " m");
        return null;
      }

      Pose2d targetPose = calculatePolePose(drive, selection.getAcceptedFaceId(), isLeftPole);
      if (targetPose == null) {
        System.out.println("Failed to calculate target pose.");
      } else {
        System.out.println("Pose calculated at runtime: " + targetPose);
      }

      return targetPose;
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

    Supplier<Pose2d> dynamicPoseSupplier =
        createDynamicCoralStationPoseSupplier(drive, distanceThresholdMeters);

    return createDriveToPose(
            drive,
            dynamicPoseSupplier,
            xJoystickSupplier,
            yJoystickSupplier,
            rotationJoystickSupplier)
        .unless(() -> dynamicPoseSupplier.get() == null);
  }

  public static Supplier<Pose2d> createDynamicCoralStationPoseSupplier(
      Drive drive, double distanceThresholdMeters) {

    return () -> {
      AlignmentUtils.CoralStationSelection selection =
          AlignmentUtils.findClosestCoralStation(drive.getPose());

      if (selection == null || selection.getAcceptedStationId() == null) {
        System.out.println("No valid coral station face found.");
        return null;
      }

      double closestDistance = selection.getAcceptedDistance();
      if (closestDistance > distanceThresholdMeters) {
        System.out.println(
            "Coral station too far: "
                + closestDistance
                + " m (limit: "
                + distanceThresholdMeters
                + ")");
        return null;
      }

      Pose2d pose = calculateCoralStationPose(drive, selection.getAcceptedStationId());
      if (pose == null) {
        System.out.println("Failed to calculate coral station pose.");
      } else {
        System.out.println("Coral station target pose: " + pose);
      }

      return pose;
    };
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
}
