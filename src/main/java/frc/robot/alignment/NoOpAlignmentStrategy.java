package frc.robot.alignment;

import edu.wpi.first.math.geometry.Translation2d;

public class NoOpAlignmentStrategy implements AlignmentStrategy {
  @Override
  public double getRotationalCorrection(AlignmentContext alignmentContext, double manualOmega) {
    // No correction, just pass through the manual input.
    return manualOmega;
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext alignmentContext, Translation2d manualTranslation) {
    // No correction, just pass through the manual translation.
    return manualTranslation;
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    return context.getRobotPose().getRotation().getRadians();
  }
}
