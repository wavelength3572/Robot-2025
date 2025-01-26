package frc.robot.commands;

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

public class DriveToPose extends Command {
  private double TIMEOUT_TIME = 8.0;
  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private double targetX;
  private double targetY;
  private double targetTheta;

  private Pose2d targetPose;
  private Pose2d currentPose;

  private Timer timeoutTimer = new Timer();

  private double driveXKp = 1.1;
  private double driveYKp = 1.1;
  private double thetaKp = 1.2;

  private final ProfiledPIDController driveControllerX =
      new ProfiledPIDController(driveXKp, 0.0, 0.0, new TrapezoidProfile.Constraints(2, .5));

  private final ProfiledPIDController driveControllerY =
      new ProfiledPIDController(driveYKp, 0.0, 0.0, new TrapezoidProfile.Constraints(2, .5));

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(360)));

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drivetrain, Pose2d pose) {
    this(drivetrain, () -> pose);
  }

    //needs to be cancellable by driver

  public DriveToPose(Drive drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    currentPose = drivetrain.getPose();
    // Reset all controllers
    driveControllerX.reset(currentPose.getX());
    driveControllerY.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(.1);
    timeoutTimer.reset();

    targetPose = poseSupplier.get();
    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetTheta = targetPose.getRotation().getRadians();
    driveControllerX.setGoal(poseSupplier.get().getX());
    driveControllerY.setGoal(poseSupplier.get().getY());
    thetaController.setGoal(poseSupplier.get().getRotation().getRadians());

    driveControllerX.setTolerance(.7);
    driveControllerY.setTolerance(1.0);
    thetaController.setTolerance(Units.degreesToRadians(.2));

    Logger.recordOutput("DriveToPose/targetPoseX", targetX);
    Logger.recordOutput("DriveToPose/targetPoseY", targetY);
    Logger.recordOutput("DriveToPose/targetPoseTheta", Units.radiansToDegrees(targetTheta));
  }

  @Override
  public void execute() {
    Logger.recordOutput("DriveToPose/XatGoal", driveControllerX.atGoal());
    Logger.recordOutput("DriveToPose/YatGoal", driveControllerY.atGoal());
    Logger.recordOutput("DriveToPose/ThetaatGoal", thetaController.atGoal());
    Logger.recordOutput("DriveToPose/driveControllerXError", driveControllerX.getPositionError());
    Logger.recordOutput("DriveToPose/driveControllerYError", driveControllerY.getPositionError());
    Logger.recordOutput("DriveToPose/thetaControllerError", thetaController.getPositionError());
    // UpdateTunableNumbers();

    // Get current and target pose
    currentPose = drivetrain.getPose();

    double currentX = currentPose.getX();
    double currentY = currentPose.getY();

    Logger.recordOutput("DriveToPose/currentX", currentX);
    Logger.recordOutput("DriveToPose/currentY", currentY);

    // Directly calculate X and Y velocities without unnecessary squaring
    double driveXVelocity = driveControllerX.calculate(currentX, targetX);
    double driveYVelocity = driveControllerY.calculate(currentY, targetY);

    // Determine the final velocities considering the drivetrain maximum speed
    // limits
    double maxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec();
    double scaledXVelocity = driveXVelocity * maxLinearSpeed;
    double scaledYVelocity = driveYVelocity * maxLinearSpeed;

    Logger.recordOutput("DriveToPose/currentTheta", currentPose.getRotation().getDegrees());

    // Calculate rotational velocity
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

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }

  @Override
  public boolean isFinished() {

    double difference =
        poseSupplier.get().getTranslation().minus(drivetrain.getPose().getTranslation()).getNorm();
    Logger.recordOutput("DriveToPose/Difference", difference);

    if (atGoal()) {
      return true;
    } else if (timeoutTimer.hasElapsed(TIMEOUT_TIME)) {
      System.out.println("Ran out of time to complete Drive to Pose");
      return true;
    }
    // else if (difference > VisionConstants.MIN_DISTANCE_FOR_DRIVE_TO_POSE){
    //   return true;
    // }
    return false;
  }
}
