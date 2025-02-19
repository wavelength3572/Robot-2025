package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class OdometryHealthMonitor {
  private static final double FROZEN_POSE_THRESHOLD = 0.05; //
  private static final double POSITION_JUMP_THRESHOLD = 1.0;
  private static final double IMU_DRIFT_THRESHOLD = 10.0;
  private static final double VISION_CONFIDENCE_THRESHOLD = 0.5;

  private Pose2d lastPose = new Pose2d();
  private double lastUpdateTime = Timer.getFPGATimestamp();
  private double odometryHealth = 1.0;
  private boolean odometryWorking = true;

  private final Drive drive;
  private final Vision vision;

  public OdometryHealthMonitor(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
  }

  public boolean checkOdometryHealth() {
    Pose2d currentPose = drive.getPose();
    Rotation2d gyroHeading = drive.getRotation();
    Rotation2d odometryHeading =
        drive.getChassisSpeeds().omegaRadiansPerSecond != 0 ? drive.getRotation() : gyroHeading;
    double currentVelocity = drive.getCurrentSpeedMetersPerSec();
    double tagConfidence = vision.isVisionOn() ? 0.8 : 0.0;

    double currentTime = Timer.getFPGATimestamp();
    boolean frozen = checkPoseFrozen(currentPose, currentVelocity);
    boolean jump = checkPositionJumps(currentPose);
    boolean imuDrift = checkIMUDrift(gyroHeading, odometryHeading);
    boolean visionFailed = tagConfidence < VISION_CONFIDENCE_THRESHOLD;

    if (frozen || jump) odometryHealth -= 0.3;
    if (imuDrift || visionFailed) odometryHealth -= 0.1;
    odometryHealth = Math.min(1.0, odometryHealth + 0.02);

    odometryWorking = odometryHealth > 0.4;
    lastPose = currentPose;
    lastUpdateTime = currentTime;

    // Logging
    Logger.recordOutput("OdometryHealth/HealthScore", odometryHealth);
    Logger.recordOutput("OdometryHealth/Working", odometryWorking);
    Logger.recordOutput("OdometryHealth/PoseFrozen", frozen);
    Logger.recordOutput("OdometryHealth/PositionJump", jump);
    Logger.recordOutput("OdometryHealth/IMUDrift", imuDrift);
    Logger.recordOutput("OdometryHealth/VisionFailed", visionFailed);
    Logger.recordOutput("OdometryHealth/CurrentPose", currentPose);
    Logger.recordOutput("OdometryHealth/GyroHeading", gyroHeading);
    Logger.recordOutput("OdometryHealth/OdometryHeading", odometryHeading);
    Logger.recordOutput("OdometryHealth/CurrentVelocity", currentVelocity);
    Logger.recordOutput("OdometryHealth/TagConfidence", tagConfidence);

    return odometryWorking;
  }

  private boolean checkPoseFrozen(Pose2d currentPose, double currentVelocity) {
    double poseDifference = lastPose.getTranslation().getDistance(currentPose.getTranslation());
    double elapsedTime = Timer.getFPGATimestamp() - lastUpdateTime;

    // Log debug values
    Logger.recordOutput("OdometryHealth/FrozenPoseDifference", poseDifference);
    Logger.recordOutput("OdometryHealth/ElapsedTimeSinceLastPose", elapsedTime);
    Logger.recordOutput("OdometryHealth/CurrentVelocity", currentVelocity);

    // Adjusted Frozen Logic
    boolean frozen =
        currentVelocity > 0.1
            && poseDifference < FROZEN_POSE_THRESHOLD
            && elapsedTime > 0.5; 

    Logger.recordOutput("OdometryHealth/PoseFrozenComputed", frozen);
    return frozen;
  }

  private boolean checkPositionJumps(Pose2d currentPose) {
    return lastPose.getTranslation().getDistance(currentPose.getTranslation())
        > POSITION_JUMP_THRESHOLD;
  }

  private boolean checkIMUDrift(Rotation2d gyroHeading, Rotation2d odometryHeading) {
    return Math.abs(gyroHeading.minus(odometryHeading).getDegrees()) > IMU_DRIFT_THRESHOLD;
  }

  public boolean isOdometryWorking() {
    return odometryWorking;
  }
}
