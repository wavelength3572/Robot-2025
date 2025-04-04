package frc.robot.commands.Alignment.TwoStage;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FinalAlign extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PPHolonomicDriveController controller;
  private int stableCycles = 0;
  private final Timer runtimeTimer = new Timer();

  // Tunable PID and tolerance values
  private static final LoggedTunableNumber kPTranslation =
      new LoggedTunableNumber("FinalAlign/kPTranslation", 7.0);
  private static final LoggedTunableNumber kDTranslation =
      new LoggedTunableNumber("FinalAlign/kDTranslation", 0.0);
  private static final LoggedTunableNumber kPRotation =
      new LoggedTunableNumber("FinalAlign/kPRotation", 1.0);
  private static final LoggedTunableNumber kDRotation =
      new LoggedTunableNumber("FinalAlign/kDRotation", 0.0);
  private static final LoggedTunableNumber rotationToleranceDeg =
      new LoggedTunableNumber("FinalAlign/RotationToleranceDeg", 1.5);
  private static final LoggedTunableNumber positionToleranceMeters =
      new LoggedTunableNumber("FinalAlign/PositionToleranceMeters", 0.02);
  private static final LoggedTunableNumber requiredStableCycles =
      new LoggedTunableNumber("FinalAlign/StableCycles", 8);

  public FinalAlign(Drive drive, Pose2d targetPose) {
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
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/TargetPose", targetPose);
    controller.reset(drive.getPose(), drive.getChassisSpeeds());
    stableCycles = 0;
    runtimeTimer.reset();
    runtimeTimer.start();
  }

  @Override
  public void execute() {
    Pose2d current = drive.getPose();
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/CurrentPose", current);

    // Create a dummy PathPlannerTrajectoryState
    PathPlannerTrajectoryState goal = new PathPlannerTrajectoryState();
    goal.pose = targetPose;
    goal.linearVelocity = 0.0;

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(drive.getPose(), goal);
    drive.runVelocity(speeds);

    // Log details
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/CommandedSpeeds", speeds);

    Pose2d error = targetPose.relativeTo(current);
    double posError = error.getTranslation().getNorm();
    double rotError = Math.abs(targetPose.getRotation().minus(current.getRotation()).getDegrees());

    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/Error/X", error.getX());
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/Error/Y", error.getY());
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/Error/RotationDeg", rotError);
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/PositionErrorMeters", posError);
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/RotationErrorDeg", rotError);

    boolean posOk = posError <= positionToleranceMeters.get();
    boolean rotOk = rotError <= rotationToleranceDeg.get();
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/WithinTolerance", posOk && rotOk);

    if (posOk && rotOk) {
      stableCycles++;
    } else {
      stableCycles = 0;
    }

    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/StableCycles", stableCycles);
  }

  @Override
  public boolean isFinished() {
    boolean done = stableCycles >= requiredStableCycles.get();
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/IsFinished", done);
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    runtimeTimer.stop();
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/DurationSeconds", runtimeTimer.get());
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/Interrupted", interrupted);
    Logger.recordOutput("AlignAndScoreTwoStage/FinalAlign/Completed", !interrupted);
  }
}
