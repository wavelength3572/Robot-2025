package frc.robot.commands.Alignment.TwoStage;

import java.util.Optional;

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

public class DriveToPosePP extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PPHolonomicDriveController controller;
  private PathPlannerTrajectory trajectory;
  private final Timer timer = new Timer();

  public DriveToPosePP(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.controller =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(1.0, 0.0, 0.0));
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d startPose = drive.getPose();

    PathConstraints constraints =
        new PathConstraints(
            1.5,
            2.0, // max vel, accel (tune these down for gentle approaches)
            Math.toRadians(180),
            Math.toRadians(360));


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

    controller.reset(startPose, drive.getChassisSpeeds());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double time = timer.get();
    if (time > trajectory.getTotalTimeSeconds()) return;

    PathPlannerTrajectoryState goal = trajectory.sample(time);
    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(drive.getPose(), goal);
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }
}
