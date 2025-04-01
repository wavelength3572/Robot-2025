package frc.robot.commands.Alignment.TwoStage;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DriveToPosePP extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PPHolonomicDriveController controller;
  private PathPlannerTrajectory trajectory;
  private final Timer timer = new Timer();
  private final Timer runtimeTimer = new Timer();
  private final boolean drivingBackwards;
  private final boolean flipStartTangent;

  // Tunable PID and motion profile constants
  private static final LoggedTunableNumber kPTranslation =
      new LoggedTunableNumber("DriveToPosePP/kPTranslation", 2.0);
  private static final LoggedTunableNumber kDTranslation =
      new LoggedTunableNumber("DriveToPosePP/kDTranslation", 0.0);
  private static final LoggedTunableNumber kPRotation =
      new LoggedTunableNumber("DriveToPosePP/kPRotation", 1.0);
  private static final LoggedTunableNumber kDRotation =
      new LoggedTunableNumber("DriveToPosePP/kDRotation", 0.0);

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("DriveToPosePP/MaxVelocity", 1.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("DriveToPosePP/MaxAcceleration", 1.5);
  private static final LoggedTunableNumber maxAngularVelocityDeg =
      new LoggedTunableNumber("DriveToPosePP/MaxAngularVelocityDeg", 180.0);
  private static final LoggedTunableNumber maxAngularAccelerationDeg =
      new LoggedTunableNumber("DriveToPosePP/MaxAngularAccelerationDeg", 360.0);

  private static final String logPrefix = "AlignAndScoreTwoStage/DriveToPosePP";

  public DriveToPosePP(
      Drive drive, Pose2d targetPose, boolean drivingBackwards, boolean flipStartTangent) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.drivingBackwards = drivingBackwards;
    this.flipStartTangent = flipStartTangent;

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

    Rotation2d desiredStartTangent =
        startPose.getRotation(); // Default to the robot's current rotation
    Rotation2d desiredEndTangent =
        targetPose.getRotation(); // Default to the target pose's rotation
    // If the robot is driving backwards, adjust adjents
    if (drivingBackwards) {
      desiredStartTangent = desiredStartTangent.plus(Rotation2d.fromDegrees(180));
      desiredEndTangent = desiredEndTangent.plus(Rotation2d.fromDegrees(180));
    }

    if (flipStartTangent) {
      desiredStartTangent = desiredStartTangent.plus(Rotation2d.fromDegrees(180));
    }

    double controlDistance = 0.4;

    trajectory =
        generateTrajectoryWithTangents(
            drive, startPose, targetPose, desiredStartTangent, desiredEndTangent, controlDistance);

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

  /**
   * Generates a PathPlannerTrajectory using custom travel tangents at the start and end. The
   * robot's actual start and target poses remain unchanged; only the spline's travel tangents are
   * adjusted via the control points.
   *
   * @param drive The drive subsystem instance.
   * @param startPose The starting Pose2d.
   * @param targetPose The target Pose2d.
   * @param startTangent The desired travel tangent (as a Rotation2d) for departure.
   * @param endTangent The desired travel tangent (as a Rotation2d) for arrival.
   * @param controlDistance The distance from the waypoint at which to place the control point.
   *     Adjust this to change how sharp or smooth the departure/arrival is.
   * @return A PathPlannerTrajectory generated with the custom tangents.
   */
  public static PathPlannerTrajectory generateTrajectoryWithTangents(
      Drive drive,
      Pose2d startPose,
      Pose2d targetPose,
      Rotation2d startTangent,
      Rotation2d endTangent,
      double controlDistance) {

    // --- Create Start Waypoint ---
    Translation2d startAnchor = startPose.getTranslation();

    // Create a unit vector from the start tangent.
    Translation2d unitStartTangent =
        new Translation2d(startTangent.getCos(), startTangent.getSin());

    // Place the next control point along the desired travel direction.
    Translation2d customStartNextControl =
        startAnchor.plus(unitStartTangent.times(controlDistance));
    Waypoint startWaypoint = new Waypoint(null, startAnchor, customStartNextControl);

    // --- Create End Waypoint ---
    Translation2d endAnchor = targetPose.getTranslation();

    // Create a unit vector from the end tangent.
    Translation2d unitEndTangent = new Translation2d(endTangent.getCos(), endTangent.getSin());

    // Place the previous control point opposite to the desired travel direction.
    Translation2d customEndPrevControl = endAnchor.minus(unitEndTangent.times(controlDistance));
    Waypoint endWaypoint = new Waypoint(customEndPrevControl, endAnchor, null);

    // --- Build Waypoints List ---
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(startWaypoint);
    waypoints.add(endWaypoint);

    // --- Define Path Constraints using tunable values ---
    double maxVel = maxVelocity.get();
    double maxAcc = maxAcceleration.get();
    double maxAngularVel = Math.toRadians(maxAngularVelocityDeg.get());
    double maxAngularAcc = Math.toRadians(maxAngularAccelerationDeg.get());
    PathConstraints constraints = new PathConstraints(maxVel, maxAcc, maxAngularVel, maxAngularAcc);

    // --- Define the Goal End State ---
    // Assume a target velocity of 0 at the endpoint.
    GoalEndState goalEndState = new GoalEndState(0, targetPose.getRotation());

    // --- Create the Custom Path ---
    PathPlannerPath customPath = new PathPlannerPath(waypoints, constraints, null, goalEndState);

    // --- Generate and Return the Trajectory ---
    return customPath.generateTrajectory(
        drive.getChassisSpeeds(), startPose.getRotation(), DriveConstants.ppConfig);
  }
}
