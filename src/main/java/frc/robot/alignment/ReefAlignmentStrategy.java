package frc.robot.alignment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants.ReefChosenOrientation;
import frc.robot.util.AlignmentUtils;

public class ReefAlignmentStrategy implements AlignmentStrategy {

  private final ProfiledPIDController angleController;

  public ReefAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {

    AlignmentUtils.ReefFaceSelection selection = context.getReefFaceSelection();
    Pose2d robotPose = context.getRobotPose();

    ReefChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForReef(robotPose, selection.getAcceptedFaceId());

    double currentAngle = robotPose.getRotation().getRadians();
    double goalAngle = chosenOrientation.rotation2D().getRadians();

    Logger.recordOutput("Alignment/ReefAlignment/CurrentAngle", Math.toDegrees(currentAngle));
    Logger.recordOutput("Alignment/ReefAlignment/GoalAngle", Math.toDegrees(goalAngle));

    return angleController.calculate(currentAngle, goalAngle);
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    // Implement reef-specific translational alignment logic
    return manualTranslation; // example
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    ReefChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForReef(
            context.getRobotPose(), context.getReefFaceSelection().getAcceptedFaceId());
    return chosenOrientation.rotation2D().getRadians();
  }
}
