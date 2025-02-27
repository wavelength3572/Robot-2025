// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.alignment.StrategyManager;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.AlignmentUtils.CageSelection;
import frc.robot.util.AlignmentUtils.CoralStationSelection;
import frc.robot.util.AlignmentUtils.ReefFaceSelection;
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RobotStatus;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private boolean isDriveModeSmart = false;
  private boolean elevatorHeightLimitsSpeed = false;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private SwerveDrivePoseEstimator poseEstimator2 =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  @Getter private ReefFaceSelection reefFaceSelection;
  @Getter private CoralStationSelection coralStationSelection;
  @Getter private Pose2d algaeTargetPose;
  @Getter private CageSelection cageSelection;

  @Getter private final StrategyManager strategyManager = new StrategyManager();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {

    Pose3d robotPose3D = convertPose2dTo3d(getPose());

    Logger.recordOutput(
        "Vision/CamPoses/FrontRightCam",
        robotPose3D.transformBy(VisionConstants.robotToFrontRightCam));
    Logger.recordOutput(
        "Vision/CamPoses/BackRightCam",
        robotPose3D.transformBy(VisionConstants.robotToBackRightCam));
    Logger.recordOutput(
        "Vision/CamPoses/FrontElevatorCam",
        robotPose3D.transformBy(VisionConstants.robotToElevatorFrontCam));
    Logger.recordOutput(
        "Vision/CamPoses/BackElevatorCam",
        robotPose3D.transformBy(VisionConstants.robotToElevatorBackCam));

    Logger.recordMetadata(getSubsystem(), getName());

    if (DriverStation.getAlliance().isPresent()) {
      reefFaceSelection = AlignmentUtils.findClosestReefFaceAndRejectOthers(getPose());
      coralStationSelection = AlignmentUtils.findClosestCoralStation(getPose());
      algaeTargetPose = AlignmentUtils.getAlgaeRemovalTargetPose(getPose(), reefFaceSelection);

      if (FieldConstants.selectedCageTranslation != null)
        cageSelection =
            AlignmentUtils.getCageAlignmentTarget(
                getPose(), FieldConstants.selectedCageTranslation);

      BranchAlignmentUtils.BranchAlignmentStatus status =
          BranchAlignmentUtils.getBranchAlignmentStatus(
              getPose(), reefFaceSelection.getAcceptedFaceId());

      switch (status) {
        case NONE:
          // Robot is not close enough to either branch; do nothing
          break;
        case RED:
          // Robot is within the outer threshold (red zone)
          break;
        case GREEN:
          // Robot is within scoring range (green zone)
          break;
      }
    }

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      poseEstimator2.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose2() {
    return poseEstimator2.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry pose. */
  public void setPose2(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(double expectedYaw) {
    // There is a delay between setting the yaw on the Pigeon and that change
    // taking effect. As a result, it is recommended to never set the yaw and
    // adjust the local offset instead.
    this.gyroIO.resetPositionToZero();
    // if (gyroInputs.connected) {
    // this.gyroOffset = expectedYaw - gyroInputs.positionDeg;
    // } else {
    // this.gyroOffset = 0;
    // this.estimatedPoseWithoutGyro =
    // new Pose2d(
    // estimatedPoseWithoutGyro.getX(),
    // estimatedPoseWithoutGyro.getY(),
    // Rotation2d.fromDegrees(expectedYaw));
    // }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    if (RobotStatus.isVisionOn()) {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurementForLogging(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    poseEstimator2.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  public void setDriveModeSmart() {
    isDriveModeSmart = true;
  }

  public void setDriveModeNormal() {
    isDriveModeSmart = false;
  }

  @AutoLogOutput(key = "Alignment/isSmartDrive")
  public boolean isDriveModeSmart() {
    return isDriveModeSmart;
  }

  public void toggleElevatorHeightLimitsSpeed() {
    elevatorHeightLimitsSpeed = !elevatorHeightLimitsSpeed;
  }

  @AutoLogOutput(key = "Alignment/elevatorHeightLimitsSpeed")
  public boolean getElevatorHeightLimitsSpeed() {
    return elevatorHeightLimitsSpeed;
  }

  public void toggleDriveMode() {
    if (isDriveModeSmart) {
      setDriveModeNormal();
    } else {
      setDriveModeSmart();
    }
  }

  /**
   * Converts a 2D pose (Pose2d) into a 3D pose (Pose3d) by assuming: - The z-coordinate is 0. -
   * Roll and pitch are 0. - Yaw comes directly from the Pose2d rotation.
   *
   * @param pose2d The 2D pose to convert.
   * @return A Pose3d representing the same pose in 3D space.
   */
  public static Pose3d convertPose2dTo3d(Pose2d pose2d) {
    // Create a 3D translation from the 2D translation; set z to 0.
    Translation3d translation3d =
        new Translation3d(
            pose2d.getTranslation().getX(),
            pose2d.getTranslation().getY(),
            0.0 // z-coordinate on the ground
            );

    // Create a 3D rotation with roll and pitch set to 0, and yaw from the 2D
    // rotation.
    Rotation3d rotation3d =
        new Rotation3d(
            0.0, // roll
            0.0, // pitch
            pose2d.getRotation().getRadians() // yaw
            );

    // Combine translation and rotation into a Pose3d.
    return new Pose3d(translation3d, rotation3d);
  }

  @AutoLogOutput(key = "Drive/CurrentSpeedMetersPerSec")
  public double getCurrentSpeedMetersPerSec() {
    // Get the current chassis speeds (these are measured from the modules).
    ChassisSpeeds speeds = getChassisSpeeds();
    // Compute the magnitude of the translational velocity.
    double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    // Round to one decimal place.
    return Math.round(speed * 10.0) / 10.0;
  }
}
