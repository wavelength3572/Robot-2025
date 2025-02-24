package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AlignmentUtils;

public class CageAlignmentStrategy implements AlignmentStrategy {
  private final ProfiledPIDController angleController;

  public CageAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {
    AlignmentUtils.CageSelection cageSelection = context.getCageSelection();
    Pose2d robotPose = context.getRobotPose();

    return angleController.calculate(
        robotPose.getRotation().getRadians(), cageSelection.getRotationToCage().getRadians());
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    return context.getCageSelection().getRotationToCage().getRadians();
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    return manualTranslation; // No translation correction in this strategy
  }
}
