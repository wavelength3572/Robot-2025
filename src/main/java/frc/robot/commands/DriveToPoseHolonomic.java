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

public class DriveToPoseHolonomic extends Command {
  private static final double TIMEOUT_TIME = 8.0;
  private static final double POSITION_TOLERANCE = 0.02; // ~3 inches tolerance

  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final double speedScalar;

  private Pose2d targetPose;
  private final Timer timeoutTimer = new Timer();

  // Controllers for X, Y, and Theta directions
  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.5, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.5));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1.5, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.5));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1.5,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(720), Math.toRadians(720)));

  public DriveToPoseHolonomic(
      Drive drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.speedScalar = MathUtil.clamp(speedScalar, 0.0, 1.0);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getPose();
    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();

    targetPose = poseSupplier.get();

    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(Units.degreesToRadians(0.1));

    timeoutTimer.reset();
    timeoutTimer.start();

    Logger.recordOutput("DriveToPoseHolonomic/targetPose", targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    // Calculate controller outputs based on current position error
    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());
    double thetaSpeed =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Clamp calculated speeds to the drivetrain's capabilities
    double effectiveMaxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec() * speedScalar;
    double effectiveMaxAngularSpeed = drivetrain.getMaxAngularSpeedRadPerSec();

    xSpeed = MathUtil.clamp(xSpeed, -effectiveMaxLinearSpeed, effectiveMaxLinearSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -effectiveMaxLinearSpeed, effectiveMaxLinearSpeed);
    thetaSpeed = MathUtil.clamp(thetaSpeed, -effectiveMaxAngularSpeed, effectiveMaxAngularSpeed);

    // Command robot movement based on the calculated speeds
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, drivetrain.getRotation());

    drivetrain.runVelocity(fieldRelativeSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();
  }

  @Override
  public boolean isFinished() {
    boolean atGoal = xController.atGoal() && yController.atGoal() && thetaController.atGoal();

    if (atGoal()) {
      Logger.recordOutput("DriveToPoseHolonomic/AtGoal", true);
      return true;
    }
    if (timeoutTimer.hasElapsed(TIMEOUT_TIME)) {
      Logger.recordOutput("DriveToPoseHolonomic/Timeout", true);
      return true;
    }
    return false;
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
