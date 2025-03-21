package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants.ReefChosenOrientation;
import frc.robot.util.AlignmentUtils;
import org.littletonrobotics.junction.Logger;

public class FlippedReefAlignmentStrategy implements AlignmentStrategy {

  private final ProfiledPIDController angleController;

  public FlippedReefAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {
    if (context.getReefFaceSelection() == null) {
      Logger.recordOutput("Alignment/ReefAlignment/Error", "No reef face selection available.");
      return manualOmega; // Return manual input if no valid selection
    }

    Pose2d robotPose = context.getRobotPose();
    AlignmentUtils.ReefFaceSelection selection = context.getReefFaceSelection();

    // Determine the normal goal rotation
    ReefChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForReef(robotPose, selection.getAcceptedFaceId());

    if (chosenOrientation == null) {
      Logger.recordOutput("Alignment/ReefAlignment/Error", "No valid reef orientation found.");
      return manualOmega;
    }

    double currentAngle = robotPose.getRotation().getRadians();
    double goalAngle =
        chosenOrientation
            .rotation2D()
            .plus(Rotation2d.fromRadians(Math.PI))
            .getRadians(); // Always flipped

    Logger.recordOutput("Alignment/ReefAlignment/CurrentAngle", Math.toDegrees(currentAngle));
    Logger.recordOutput("Alignment/ReefAlignment/GoalAngle", Math.toDegrees(goalAngle));

    return angleController.calculate(currentAngle, goalAngle);
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    return manualTranslation; // No additional translation corrections needed
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    if (context.getReefFaceSelection() == null) {
      return context.getRobotPose().getRotation().getRadians();
    }

    ReefChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForReef(
            context.getRobotPose(), context.getReefFaceSelection().getAcceptedFaceId());

    if (chosenOrientation == null) {
      return context.getRobotPose().getRotation().getRadians();
    }

    return chosenOrientation
        .rotation2D()
        .plus(Rotation2d.fromRadians(Math.PI))
        .getRadians(); // Always flipped
  }
}
