package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.RobotStatus;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AlignToReefPP extends Command {

  private static final double FORWARD_THRESHOLD_METERS = Units.inchesToMeters(3);
  private static final double LATERAL_THRESHOLD_METERS = Units.inchesToMeters(1);
  private static final double ROTATION_THRESHOLD_DEGREES = 0.5;
  private static final int REQUIRED_STABLE_CYCLES = 2;
  private static final double MAX_RUNTIME_SECONDS = 1.5; // Timeout to prevent infinite execution

  private int stableCycles = 0;
  private final Drive drivetrain;
  private final PPHolonomicDriveController holonomicController;
  private PathPlannerTrajectory trajectory;
  private final Timer timer = new Timer();
  private final Timer finalCorrectionTimer = new Timer();
  private boolean trajectoryGenerated = false;
  private boolean finalCorrectionStarted = false;
  private final boolean isLeftPole;
  private boolean timedOut = false;

  public AlignToReefPP(Drive drivetrain, boolean isLeftPole) {
    this.drivetrain = drivetrain;
    this.holonomicController =
        new PPHolonomicDriveController(
            new PIDConstants(2.0, 0.0, 0), new PIDConstants(2.0, 0.0, 0));
    this.isLeftPole = isLeftPole;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getPose();
    Logger.recordOutput("AlignToReef/Poses/InitialPose", currentPose);

    int reefFaceId = RobotStatus.getReefFaceSelection().getAcceptedFaceId();
    Logger.recordOutput("AlignToReef/ReefFaceId", reefFaceId);

    Pose2d targetPose = getOffsetTargetPose(drivetrain, reefFaceId, isLeftPole);
    if (targetPose == null) {
      Logger.recordOutput("AlignToReef/Error", "No valid target pose. Cancelling.");
      trajectoryGenerated = false;
      return;
    }
    Logger.recordOutput("AlignToReef/Poses/TargetPose", targetPose);

    PathConstraints constraints =
        new PathConstraints(2.0, 3.0, Math.toRadians(720), Math.toRadians(720));

    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(currentPose, targetPose),
            constraints,
            null,
            new GoalEndState(0.0, targetPose.getRotation()));

    path.preventFlipping = true;

    trajectory =
        path.generateTrajectory(
            drivetrain.getChassisSpeeds(), currentPose.getRotation(), DriveConstants.ppConfig);

    // **Trajectory Logging Restored**
    Pose2d[] trajectoryPoses =
        trajectory.getStates().stream().map(state -> state.pose).toArray(Pose2d[]::new);
    Logger.recordOutput("AlignToReef/Trajectory", trajectoryPoses);
    Logger.recordOutput("AlignToReef/TrajectoryDuration", trajectory.getTotalTimeSeconds());

    Logger.recordOutput("AlignToReef/TrajectoryGenerated", true);

    trajectoryGenerated = true;
    stableCycles = 0;
    timer.reset();
    timer.start();
    finalCorrectionStarted = false;
    timedOut = false;

    holonomicController.reset(currentPose, drivetrain.getChassisSpeeds());
  }

  @Override
  public void execute() {
    if (!trajectoryGenerated) {
      Logger.recordOutput("AlignToReef/Error", "No trajectory generated.");
      drivetrain.stop();
      return;
    }

    double elapsedTime = timer.get();
    Logger.recordOutput("AlignToReef/Summary/ElapsedTime", elapsedTime);

    if (elapsedTime >= MAX_RUNTIME_SECONDS) {
      timedOut = true;
      return;
    }

    if (elapsedTime < trajectory.getTotalTimeSeconds()) {
      PathPlannerTrajectoryState goalState = trajectory.sample(elapsedTime);
      ChassisSpeeds speeds =
          holonomicController.calculateRobotRelativeSpeeds(drivetrain.getPose(), goalState);
      drivetrain.runVelocity(speeds);

      Logger.recordOutput("AlignToReef/State", "Following Trajectory");
      return;
    }

    if (!finalCorrectionStarted) {
      finalCorrectionTimer.reset();
      finalCorrectionTimer.start();
      finalCorrectionStarted = true;
    }

    Pose2d currentPose = drivetrain.getPose();
    PathPlannerTrajectoryState finalState = trajectory.getEndState();

    // Compute relative pose
    Pose2d relativePose = currentPose.relativeTo(finalState.pose);
    double forwardError = Units.metersToInches(relativePose.getTranslation().getX());
    double lateralError = Units.metersToInches(relativePose.getTranslation().getY());
    double rotationError =
        Math.abs(currentPose.getRotation().minus(finalState.pose.getRotation()).getDegrees());

    // Log errors for debugging
    Logger.recordOutput("AlignToReef/Summary/ForwardErrorInches", forwardError);
    Logger.recordOutput("AlignToReef/Summary/LateralErrorInches", lateralError);
    Logger.recordOutput("AlignToReef/Summary/RotationErrorDegrees", rotationError);

    // Define separate tolerances
    boolean withinForwardTolerance = Math.abs(forwardError) <= FORWARD_THRESHOLD_METERS;
    boolean withinLateralTolerance = Math.abs(lateralError) <= LATERAL_THRESHOLD_METERS;
    boolean withinRotationTolerance = rotationError <= ROTATION_THRESHOLD_DEGREES;

    if (!withinForwardTolerance || !withinLateralTolerance || !withinRotationTolerance) {
      Logger.recordOutput("AlignToReef/State", "Final Correction");
      ChassisSpeeds correctionSpeeds =
          holonomicController.calculateRobotRelativeSpeeds(currentPose, finalState);
      drivetrain.runVelocity(correctionSpeeds);
      stableCycles = 0;
    } else {
      stableCycles++;
      Logger.recordOutput("AlignToReef/State", "Stable");
      Logger.recordOutput("AlignToReef/StableCycles", stableCycles);
      drivetrain.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timer.stop();
    finalCorrectionTimer.stop();

    // Log the final pose when the command ends
    Pose2d finalPose = drivetrain.getPose();
    Logger.recordOutput("AlignToReef/Poses/FinalPose", finalPose);

    if (timedOut) {
      Logger.recordOutput("AlignToReef/Summary/CompletionStatus", "Timed Out");
    } else if (interrupted) {
      Logger.recordOutput("AlignToReef/Summary/CompletionStatus", "Interrupted");
    } else {
      Logger.recordOutput("AlignToReef/Summary/CompletionStatus", "Completed Successfully");
    }

    // Only log correction time if final correction started
    if (finalCorrectionStarted) {
      Logger.recordOutput("AlignToReef/Summary/FinalCorrectionTime", finalCorrectionTimer.get());
    }
  }

  @Override
  public boolean isFinished() {
    boolean finished = !trajectoryGenerated || stableCycles >= REQUIRED_STABLE_CYCLES || timedOut;
    Logger.recordOutput("AlignToReef/IsFinished", finished);
    return finished;
  }

  private Pose2d getOffsetTargetPose(Drive drive, int reefFaceId, boolean isLeftBranch) {
    Optional<Pose3d> reefFacePose3d = VisionConstants.aprilTagLayout.getTagPose(reefFaceId);
    if (reefFacePose3d.isEmpty()) {
      Logger.recordOutput("AlignToReef/Error", "No valid reef face tag found.");
      return null;
    }

    boolean isScoringL1 = RobotStatus.getTargetPreset() == CoralSystemPresets.L1_SCORE;

    return isScoringL1
        ? DriveToCommands.calculateL1Pose(drive, reefFaceId)
        : DriveToCommands.calculatePolePose(drive, reefFaceId, isLeftBranch);
  }
}
