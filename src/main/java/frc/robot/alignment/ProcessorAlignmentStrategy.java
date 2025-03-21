package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AlignmentUtils;
import org.littletonrobotics.junction.Logger;

public class ProcessorAlignmentStrategy implements AlignmentStrategy {
  private final ProfiledPIDController angleController;

  public ProcessorAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {
    AlignmentUtils.ProcessorSelection selection = context.getProcessorSelection();
    Pose2d robotPose = context.getRobotPose();

    double currentAngle = robotPose.getRotation().getRadians();
    double goalAngle = selection.getRotationToProcessor().getRadians();

    Logger.recordOutput("Alignment/ProcessorAlignment/CurrentAngle", Math.toDegrees(currentAngle));
    Logger.recordOutput("Alignment/ProcessorAlignment/GoalAngle", Math.toDegrees(goalAngle));

    return angleController.calculate(currentAngle, goalAngle);
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    // Implement processor-specific translational alignment logic
    return manualTranslation; // No translation correction for now
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    AlignmentUtils.ProcessorSelection selection = context.getProcessorSelection();
    return selection.getRotationToProcessor().getRadians();
  }
}
