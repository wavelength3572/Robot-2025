package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseNoJoystick extends Command {
  private static final double TIMEOUT_TIME = 3.25; // seconds
  private static final double HEADING_STABILITY_THRESHOLD = 0.3; // rad/sec

  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final double speedScalar;

  private final ProfiledPIDController driveControllerX =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.2, 4.0));
  private final ProfiledPIDController driveControllerY =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.2, 4.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          .8,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(180), Math.toRadians(360)));

  private Pose2d targetPose;
  private boolean initializedControllers = false;
  private final Timer timeoutTimer = new Timer();

  public DriveToPoseNoJoystick(Drive drivetrain, Supplier<Pose2d> poseSupplier, double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.speedScalar = MathUtil.clamp(speedScalar, 0.0, 1.0);

    if (speedScalar != this.speedScalar) {
      Logger.recordOutput("DriveToPose/Warning",
          "Speed scalar " + speedScalar + " out of bounds; clamped to " + this.speedScalar);
    }

    addRequirements(drivetrain);
  }

  public DriveToPoseNoJoystick(Drive drivetrain, Supplier<Pose2d> poseSupplier) {
    this(drivetrain, poseSupplier, 1.0);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPose/Init/SpeedScalar", speedScalar);
    Logger.recordOutput("DriveToPose/Init/TimeoutTime", TIMEOUT_TIME);
    timeoutTimer.restart();
    initializedControllers = false;
  }

  @Override
  public void execute() {
    if (!initializedControllers) {
      if (Math.abs(drivetrain.getChassisSpeeds().omegaRadiansPerSecond) < HEADING_STABILITY_THRESHOLD) {
        Pose2d currentPose = drivetrain.getPose();
        targetPose = poseSupplier.get();

        driveControllerX.reset(currentPose.getX());
        driveControllerY.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        driveControllerX.setGoal(targetPose.getX());
        driveControllerY.setGoal(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());

        driveControllerX.setTolerance(Units.inchesToMeters(0.5));
        driveControllerY.setTolerance(Units.inchesToMeters(0.5));
        thetaController.setTolerance(Units.degreesToRadians(0.1));

        Logger.recordOutput("DriveToPose/Init/StartPose", currentPose);
        Logger.recordOutput("DriveToPose/Init/TargetPose", targetPose);
        initializedControllers = true;
      } else {
        Logger.recordOutput("DriveToPose/WaitingForHeadingStability", true);
        drivetrain.stop();
        return;
      }
    }

    Pose2d currentPose = drivetrain.getPose();

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    Logger.recordOutput("DriveToPose/Error/XErrorMeters", xError);
    Logger.recordOutput("DriveToPose/Error/YErrorMeters", yError);
    Logger.recordOutput("DriveToPose/Error/ThetaErrorDegrees", Units.radiansToDegrees(thetaError));
    Logger.recordOutput("DriveToPose/RobotHeadingDegrees",
        Units.radiansToDegrees(currentPose.getRotation().getRadians()));

    double driveXVelocity = driveControllerX.calculate(currentPose.getX(), targetPose.getX());
    double driveYVelocity = driveControllerY.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double effectiveMaxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec() * speedScalar;
    double scaledXVelocity = driveXVelocity * effectiveMaxLinearSpeed;
    double scaledYVelocity = driveYVelocity * effectiveMaxLinearSpeed;
    double scaledThetaVelocity = thetaVelocity * drivetrain.getMaxAngularSpeedRadPerSec();

    if (thetaController.atGoal()) scaledThetaVelocity = 0.0;

    Logger.recordOutput("DriveToPose/VelocityCommands/X", scaledXVelocity);
    Logger.recordOutput("DriveToPose/VelocityCommands/Y", scaledYVelocity);
    Logger.recordOutput("DriveToPose/VelocityCommands/Theta",
        Units.radiansToDegrees(scaledThetaVelocity));

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            scaledXVelocity, scaledYVelocity, scaledThetaVelocity, drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();

    Logger.recordOutput("DriveToPose/End/FinalPose", drivetrain.getPose());
    Logger.recordOutput("DriveToPose/End/TotalTime", timeoutTimer.get());
    Logger.recordOutput("DriveToPose/End/WasInterrupted", interrupted);
    Logger.recordOutput("DriveToPose/End/TimedOut", timeoutTimer.hasElapsed(TIMEOUT_TIME));
  }

  @Override
  public boolean isFinished() {
    boolean finished = initializedControllers && (atGoal() || timeoutTimer.hasElapsed(TIMEOUT_TIME));

    Logger.recordOutput("DriveToPose/IsFinished", finished);
    Logger.recordOutput("DriveToPose/CompletionReason", atGoal() ? "Reached Target" : "Timed Out");

    return finished;
  }

  private boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }
}