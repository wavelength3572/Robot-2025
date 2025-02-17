package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants.StationChosenOrientation;
import frc.robot.util.AlignmentUtils;

public class CoralStationAlignmentStrategy implements AlignmentStrategy {
  private final ProfiledPIDController angleController;

  public CoralStationAlignmentStrategy(ProfiledPIDController sharedAngleController) {
    this.angleController = sharedAngleController;
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualOmega) {
    AlignmentUtils.CoralStationSelection selection = context.getCoralStationSelection();
    Pose2d robotPose = context.getRobotPose();

    // Select the best coral station face to align with
    StationChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForStation(
            robotPose, selection.getAcceptedStationId());

    return angleController.calculate(
        robotPose.getRotation().getRadians(), chosenOrientation.rotation2D().getRadians());
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    // Implement coral station-specific translational alignment logic
    return manualTranslation; // No translation correction for now
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    StationChosenOrientation chosenOrientation =
        AlignmentUtils.pickClosestOrientationForStation(
            context.getRobotPose(), context.getCoralStationSelection().getAcceptedStationId());
    return chosenOrientation.rotation2D().getRadians();
  }
}
