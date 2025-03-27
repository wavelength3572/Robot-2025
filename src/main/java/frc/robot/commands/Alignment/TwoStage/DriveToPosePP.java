package frc.robot.commands.Alignment.TwoStage;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToPosePP extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PPHolonomicDriveController controller;
  private PathPlannerTrajectory trajectory;
  private final Timer timer = new Timer();
  private final Timer runtimeTimer = new Timer();

  // Tunable PID and motion profile constants
  private static final LoggedTunableNumber kPTranslation =
      new LoggedTunableNumber("DriveToPosePP/kPTranslation", 5.0);
  private static final LoggedTunableNumber kDTranslation =
      new LoggedTunableNumber("DriveToPosePP/kDTranslation", 0.0);
  private static final LoggedTunableNumber kPRotation =
      new LoggedTunableNumber("DriveToPosePP/kPRotation", 1.0);
  private static final LoggedTunableNumber kDRotation =
      new LoggedTunableNumber("DriveToPosePP/kDRotation", 0.0);

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("DriveToPosePP/MaxVelocity", 1.5);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("DriveToPosePP/MaxAcceleration", 2.0);
  private static final LoggedTunableNumber maxAngularVelocityDeg =
      new LoggedTunableNumber("DriveToPosePP/MaxAngularVelocityDeg", 180.0);
  private static final LoggedTunableNumber maxAngularAccelerationDeg =
      new LoggedTunableNumber("DriveToPosePP/MaxAngularAccelerationDeg", 360.0);

  private static final String logPrefix = "AlignAndScoreTwoStage/DriveToPosePP";

  public DriveToPosePP(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;

    this.controller =
        new PPHolonomicDriveController(
            new PIDConstants(kPTranslation.get(), 0.0, kDTranslation.get()),
            new PIDConstants(kPRotation.get(), 0.0, kDRotation.get()));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d startPose = drive.getPose();
    Logger.recordOutput(logPrefix + "/StartPose", startPose);
    Logger.recordOutput(logPrefix + "/TargetPose", targetPose);

    PathConstraints constraints =
        new PathConstraints(
            maxVelocity.get(),
            maxAcceleration.get(),
            Math.toRadians(maxAngularVelocityDeg.get()),
            Math.toRadians(maxAngularAccelerationDeg.get()));

    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(startPose, targetPose),
            constraints,
            null,
            new GoalEndState(0.0, targetPose.getRotation()));
    path.preventFlipping = true;

    trajectory =
        path.generateTrajectory(
            drive.getChassisSpeeds(), startPose.getRotation(), DriveConstants.ppConfig);

    Logger.recordOutput(logPrefix + "/TrajectoryDuration", trajectory.getTotalTimeSeconds());
    Logger.recordOutput(logPrefix + "/TrajectoryPointCount", trajectory.getStates().size());

    Pose2d[] poses =
        trajectory.getStates().stream().map(state -> state.pose).toArray(Pose2d[]::new);
    Logger.recordOutput(logPrefix + "/TrajectoryPoses", poses);

    controller.reset(startPose, drive.getChassisSpeeds());
    timer.reset();
    timer.start();
    runtimeTimer.reset();
    runtimeTimer.start();
  }

  @Override
  public void execute() {
    double time = timer.get();
    Logger.recordOutput(logPrefix + "/ElapsedTime", time);

    if (time > trajectory.getTotalTimeSeconds()) return;

    PathPlannerTrajectoryState goal = trajectory.sample(time);
    Logger.recordOutput(logPrefix + "/GoalPose", goal.pose);
    Logger.recordOutput(logPrefix + "/GoalVelocity", goal.linearVelocity);

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(drive.getPose(), goal);
    Logger.recordOutput(logPrefix + "/CommandedSpeeds", speeds);

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
    runtimeTimer.stop();

    Logger.recordOutput(logPrefix + "/DurationSeconds", runtimeTimer.get());
    Logger.recordOutput(logPrefix + "/Interrupted", interrupted);
    Logger.recordOutput(logPrefix + "/Finished", !interrupted);
  }

  @Override
  public boolean isFinished() {
    boolean done = timer.get() >= trajectory.getTotalTimeSeconds();
    Logger.recordOutput(logPrefix + "/IsFinished", done);
    return done;
  }
}
