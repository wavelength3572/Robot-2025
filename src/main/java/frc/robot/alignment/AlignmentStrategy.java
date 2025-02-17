package frc.robot.alignment;

import edu.wpi.first.math.geometry.Translation2d;

public interface AlignmentStrategy {
  /**
   * Computes a rotational correction.
   *
   * @param alignmentContext The current alignment context containing state like robot pose.
   * @param manualOmega The manual rotational input.
   * @return The corrected rotational command.
   */
  double getRotationalCorrection(AlignmentContext alignmentContext, double manualOmega);

  /**
   * Computes a translational correction.
   *
   * @param alignmentContext The current alignment context containing state like robot pose.
   * @param manualTranslation The manual translation command.
   * @return The corrected translation command.
   */
  Translation2d getTranslationalCorrection(
      AlignmentContext alignmentContext, Translation2d manualTranslation);

  /** Returns the goal rotation (in radians) for this alignment strategy. */
  double getGoalRotation(AlignmentContext context);
}
