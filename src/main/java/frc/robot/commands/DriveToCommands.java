package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandConstants.ReefChosenOrientation;
import frc.robot.commands.CommandConstants.ReefFacesBlue;
import frc.robot.commands.CommandConstants.ReefFacesRed;
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
              ? chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? blueFace.getLeftPole().getFrontTranslation()
                  : blueFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
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
              ? chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? redFace.getLeftPole().getFrontTranslation()
                  : redFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? redFace.getRightPole().getFrontTranslation()
                  : redFace.getRightPole().getBackTranslation();

      // Return the pose with proper orientation
      return new Pose2d(poleTranslation, chosen.rotation2D());

    } else {
      System.out.println("Unknown alliance. Cannot calculate target pose.");
      return null;
    }
  }
}
