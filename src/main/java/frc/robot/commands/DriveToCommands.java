package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandConstants.ChosenOrientation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefAlignmentUtils;
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
          ReefAlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();

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
          Pose2d targetPose = calculatePolePose(drive, isLeftPole);
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

  public static Pose2d calculatePolePose(Drive drive, boolean isLeftPole) {
    // 1) Get the ID of the closest reef face
    ReefAlignmentUtils.ReefFaceSelection selection = drive.getReefFaceSelection();
    Integer faceId = selection.getAcceptedFaceId();
    if (faceId == null) {
      System.out.println("No valid reef face found. Cannot calculate pole pose.");
      return null;
    }

    // 2) Check which alliance we're on
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // 3) Pick the best orientation for the face
    ChosenOrientation chosen =
        ReefAlignmentUtils.pickClosestOrientationForFace(drive.getPose(), faceId);

    // 4) Use the chosen orientation and faceId to calculate the target pose
    return ReefAlignmentUtils.calculateTargetPose(drive, faceId, isLeftPole, alliance, chosen);
  }
}
