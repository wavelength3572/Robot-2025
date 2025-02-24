package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.util.AlignmentUtils;
import org.littletonrobotics.junction.Logger;

public class CageFullAlignmentStrategy implements AlignmentStrategy {
  private final ProfiledPIDController angleController;

  // Tuning parameters
  private static final double KP_LATERAL = .8; // Adjusted for smoother correction
  private static final double LATERAL_ERROR_THRESHOLD =
      0.09; // Increased for smoother driver assist
  private static final double FORWARD_THRESHOLD = 1.2; // Adjusted distance for forward speed assist
  private static final double MIN_FORWARD_SCALE = 0.22; // Minimum forward scaling factor

  public CageFullAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {
    AlignmentUtils.CageSelection cageSelection = context.getCageSelection();

    double currentAngle = context.getRobotPose().getRotation().getRadians();
    double goalAngle = cageSelection.getRotationToCage().getRadians();

    Logger.recordOutput("Alignment/ReefAlignment/CurrentAngle", Math.toDegrees(currentAngle));
    Logger.recordOutput("Alignment/ReefAlignment/GoalAngle", Math.toDegrees(goalAngle));

    return angleController.calculate(currentAngle, goalAngle);
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    return context.getCageSelection().getRotationToCage().getRadians();
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    AlignmentUtils.CageSelection cageSelection = context.getCageSelection();
    Pose2d climberTipPose = ClimberConstants.computeClimberTipPose(context.getRobotPose());

    // Compute lateral error (difference in Y) **using climber tip pose**
    double lateralError = cageSelection.getCageOpeningPose2d().getY() - climberTipPose.getY();

    // Apply proportional correction with threshold
    double correctiveLateral = KP_LATERAL * lateralError;
    correctiveLateral =
        applyBlendedCorrection(manualTranslation.getY(), correctiveLateral, lateralError);

    // Scale forward speed based on distance to cage
    double distanceToCage = cageSelection.getDistanceToCage();
    double forwardScaling =
        computeForwardScaling(distanceToCage, FORWARD_THRESHOLD, MIN_FORWARD_SCALE);
    double scaledForward = manualTranslation.getX() * forwardScaling;

    return new Translation2d(scaledForward, correctiveLateral);
  }

  /** Blends driver lateral input with corrective lateral speed based on error threshold. */
  private double applyBlendedCorrection(
      double driverLateral, double correctiveLateral, double lateralError) {
    double autoGain = Math.min(1.0, Math.abs(lateralError) / LATERAL_ERROR_THRESHOLD);
    return (1 - autoGain) * driverLateral + autoGain * correctiveLateral;
  }

  /** Computes a forward speed scaling factor based on proximity to the cage. */
  private double computeForwardScaling(double distanceToCage, double threshold, double minScale) {
    double scale = Math.min(1.0, Math.abs(distanceToCage) / threshold);
    return Math.max(scale, minScale);
  }
}
