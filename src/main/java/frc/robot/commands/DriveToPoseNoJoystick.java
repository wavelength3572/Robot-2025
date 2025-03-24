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
  private static final double TIMEOUT_TIME = 1.5; // Timeout to prevent infinite execution

  private final double speedScalar;
  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private Pose2d targetPose;
  private Pose2d currentPose;
  private Timer timeoutTimer = new Timer();

  private final ProfiledPIDController driveControllerX =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.2, 4.0));
  private final ProfiledPIDController driveControllerY =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.2, 4.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1.2,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(720), Math.toRadians(720)));

  /** Drives to the specified pose under full software control. */
  public DriveToPoseNoJoystick(
      Drive drivetrain, Supplier<Pose2d> poseSupplier, double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;

    // Clamp the speedScalar between 0 and 1 to prevent unsafe values.
    double clampedScalar = MathUtil.clamp(speedScalar, 0.0, 1.0);
    if (clampedScalar != speedScalar) {
      Logger.recordOutput(
          "DriveToPose/Warning",
          "Speed scalar " + speedScalar + " out of bounds; clamped to " + clampedScalar);
    }
    this.speedScalar = clampedScalar;
    addRequirements(drivetrain);
  }

  public DriveToPoseNoJoystick(Drive drivetrain, Supplier<Pose2d> poseSupplier) {
    this(drivetrain, poseSupplier, 1.0);
  }

  @Override
  public void initialize() {
    currentPose = drivetrain.getPose();
    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();

    // Log initial state
    Logger.recordOutput("DriveToPose/Init/StartPose", currentPose);
    Logger.recordOutput("DriveToPose/Init/SpeedScalar", speedScalar);
    Logger.recordOutput("DriveToPose/Init/TimeoutTime", TIMEOUT_TIME);

    driveControllerX.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    driveControllerY.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose = poseSupplier.get();
    driveControllerX.setGoal(targetPose.getX());
    driveControllerY.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    // Log target pose
    Logger.recordOutput("DriveToPose/Init/TargetPose", targetPose);

    driveControllerX.setTolerance(Units.inchesToMeters(.5));
    driveControllerY.setTolerance(Units.inchesToMeters(.5));
    thetaController.setTolerance(Units.degreesToRadians(0.1));

    timeoutTimer.restart();
  }

  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    // Calculate errors
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError =
        targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    Logger.recordOutput("DriveToPose/Error/XErrorMeters", xError);
    Logger.recordOutput("DriveToPose/Error/YErrorMeters", yError);
    Logger.recordOutput("DriveToPose/Error/ThetaErrorDegrees", Units.radiansToDegrees(thetaError));

    // Calculate velocities
    double driveXVelocity = driveControllerX.calculate(currentPose.getX(), targetPose.getX());
    double driveYVelocity = driveControllerY.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Scale velocities
    double effectiveMaxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec() * speedScalar;
    double scaledXVelocity = driveXVelocity * effectiveMaxLinearSpeed;
    double scaledYVelocity = driveYVelocity * effectiveMaxLinearSpeed;
    double scaledThetaVelocity = thetaVelocity * drivetrain.getMaxAngularSpeedRadPerSec();

    if (thetaController.atGoal()) scaledThetaVelocity = 0.0;

    Logger.recordOutput("DriveToPose/VelocityCommands/X", scaledXVelocity);
    Logger.recordOutput("DriveToPose/VelocityCommands/Y", scaledYVelocity);
    Logger.recordOutput(
        "DriveToPose/VelocityCommands/Theta", Units.radiansToDegrees(scaledThetaVelocity));

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            scaledXVelocity, scaledYVelocity, scaledThetaVelocity, drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();

    // Log completion state
    Logger.recordOutput("DriveToPose/End/FinalPose", drivetrain.getPose());
    Logger.recordOutput("DriveToPose/End/TotalTime", timeoutTimer.get());
    Logger.recordOutput("DriveToPose/End/WasInterrupted", interrupted);
    Logger.recordOutput("DriveToPose/End/TimedOut", timeoutTimer.hasElapsed(TIMEOUT_TIME));
  }

  @Override
  public boolean isFinished() {
    boolean finished = atGoal() || timeoutTimer.hasElapsed(TIMEOUT_TIME);

    Logger.recordOutput("DriveToPose/IsFinished", finished);
    Logger.recordOutput("DriveToPose/CompletionReason", atGoal() ? "Reached Target" : "Timed Out");

    return finished;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }
}
