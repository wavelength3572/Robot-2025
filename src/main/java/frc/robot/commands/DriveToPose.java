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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final double JOYSTICK_DEADBAND = 0.1; // Adjust as needed
  private double TIMEOUT_TIME = 8.0;

  private final double speedScalar;

  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final DoubleSupplier xJoystickSupplier;
  private final DoubleSupplier yJoystickSupplier;
  private final DoubleSupplier rotationJoystickSupplier;

  private double targetX;
  private double targetY;
  private double targetTheta;

  private Pose2d targetPose;
  private Pose2d currentPose;

  private Timer timeoutTimer = new Timer();

  private double driveXKp = .9; // changed from 1.1
  private double driveYKp = .9; // changed from 1.1
  private double thetaKp = 0.9;

  private final ProfiledPIDController driveControllerX =
      new ProfiledPIDController(
          driveXKp,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(2, 2)); // upped accelleration from .5

  private final ProfiledPIDController driveControllerY =
      new ProfiledPIDController(
          driveYKp,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(2, 2)); // upped accelleration from .5

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(360)));

  /** Drives to the specified pose under full software control. */
  public DriveToPose(
      Drive drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.xJoystickSupplier = xJoystickSupplier;
    this.yJoystickSupplier = yJoystickSupplier;
    this.rotationJoystickSupplier = rotationJoystickSupplier;

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

  public DriveToPose(
      Drive drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier) {
    this(
        drivetrain,
        poseSupplier,
        xJoystickSupplier,
        yJoystickSupplier,
        rotationJoystickSupplier,
        1.0);
  }

  @Override
  public void initialize() {
    currentPose = drivetrain.getPose();
    driveControllerX.reset(currentPose.getX());
    driveControllerY.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timeoutTimer.reset();

    targetPose = poseSupplier.get();
    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetTheta = targetPose.getRotation().getRadians();
    driveControllerX.setGoal(targetX);
    driveControllerY.setGoal(targetY);
    thetaController.setGoal(targetTheta);

    driveControllerX.setTolerance(0.0762); // 3 inches tolerance, was much larger
    driveControllerY.setTolerance(0.0762); // 3 inches tolerance, was much larger
    thetaController.setTolerance(Units.degreesToRadians(.1));

    Logger.recordOutput("DriveToPose/targetPoseX", targetX);
    Logger.recordOutput("DriveToPose/targetPoseY", targetY);
    Logger.recordOutput("DriveToPose/targetPoseTheta", Units.radiansToDegrees(targetTheta));
  }

  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    double driveXVelocity = driveControllerX.calculate(currentPose.getX(), targetX);
    double driveYVelocity = driveControllerY.calculate(currentPose.getY(), targetY);

    double effectiveMaxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec() * speedScalar;

    double scaledXVelocity = driveXVelocity * effectiveMaxLinearSpeed;
    double scaledYVelocity = driveYVelocity * effectiveMaxLinearSpeed;

    double thetaVelocity =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetTheta);
    double maxAngularSpeed = drivetrain.getMaxAngularSpeedRadPerSec();
    double scaledThetaVelocity = thetaVelocity * maxAngularSpeed;

    if (thetaController.atGoal()) scaledThetaVelocity = 0.0;

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            scaledXVelocity, scaledYVelocity, scaledThetaVelocity, drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if the driver moves either joystick or rotates the robot
    if (Math.abs(xJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(yJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(rotationJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND) {
      Logger.recordOutput("DriveToPose/CanceledByJoystick", true);
      return true;
    }

    // Check if the robot has reached the goal
    if (atGoal()) {
      return true;
    }

    // Check if the command has timed out
    if (timeoutTimer.hasElapsed(TIMEOUT_TIME)) {
      Logger.recordOutput("DriveToPose/Timeout", true);
      return true;
    }

    return false;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }
}
