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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.alignment.AlignmentContext;
import frc.robot.alignment.StrategyManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.RobotStatus;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double rawMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    double scaledMagnitude = rawMagnitude * rawMagnitude; // Squared for sensitivity.

    // Create the vector from the scaled magnitude and direction.
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));
    Translation2d vector =
        new Pose2d(new Translation2d(), direction)
            .transformBy(new Transform2d(new Translation2d(scaledMagnitude, 0.0), new Rotation2d()))
            .getTranslation();

    // If the magnitude is greater than 1, normalize it.
    if (vector.getNorm() > 1.0) {
      vector = vector.times(1.0 / vector.getNorm());
    }

    return vector;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          sendSpeedsToDrive(drive, linearVelocity, omega);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              sendSpeedsToDrive(drive, linearVelocity, omega);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static InstantCommand toggleSmartDriveCmd(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Boolean> haveCoralSupplier,
      Supplier<Boolean> haveAlgaeSupplier,
      Supplier<Boolean> climberDeployedSupplier,
      DoubleSupplier elevatorHeightInchesSupplier) {
    return new InstantCommand(
        () -> {
          if (drive.isDriveModeSmart()) {
            drive.setDriveModeNormal();
            drive.setDefaultCommand(
                DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier));
          } else {
            drive.setDriveModeSmart();
            drive.setDefaultCommand(
                JoystickAlignmentStrategyDrive(
                    drive,
                    xSupplier,
                    ySupplier,
                    omegaSupplier,
                    drive::getPose,
                    elevatorHeightInchesSupplier,
                    haveCoralSupplier,
                    haveAlgaeSupplier,
                    climberDeployedSupplier,
                    drive::getReefFaceSelection,
                    drive::getCoralStationSelection,
                    drive::getCageSelection,
                    drive::getProcessorSelection));
          }
        },
        drive);
  }

  public static InstantCommand setSmartDriveCmd(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Boolean> haveCoralSupplier,
      Supplier<Boolean> haveAlgaeSupplier,
      Supplier<Boolean> climberDeployedSupplier,
      DoubleSupplier elevatorHeightInchesSupplier) {
    return new InstantCommand(
        () -> {
          drive.setDriveModeSmart();
          drive.setDefaultCommand(
              JoystickAlignmentStrategyDrive(
                  drive,
                  xSupplier,
                  ySupplier,
                  omegaSupplier,
                  drive::getPose,
                  elevatorHeightInchesSupplier,
                  haveCoralSupplier,
                  haveAlgaeSupplier,
                  climberDeployedSupplier,
                  drive::getReefFaceSelection,
                  drive::getCoralStationSelection,
                  drive::getCageSelection,
                  drive::getProcessorSelection));
        },
        drive);
  }

  public static InstantCommand setNormalDriveCmd(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return new InstantCommand(
        () -> {
          drive.setDriveModeNormal();
          drive.setDefaultCommand(
              DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier));
        },
        drive);
  }

  /** Creates a joystick-based drive command that integrates alignment strategies. */
  public static Command JoystickAlignmentStrategyDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier elevatorHeightInchesSupplier,
      Supplier<Boolean> haveCoralSupplier,
      Supplier<Boolean> haveAlgaeSupplier,
      Supplier<Boolean> climberDeployedSupplier,
      Supplier<AlignmentUtils.ReefFaceSelection> reefFaceSelectionSupplier,
      Supplier<AlignmentUtils.CoralStationSelection> coralStationSelectionSupplier,
      Supplier<AlignmentUtils.CageSelection> cageSelectionSupplier,
      Supplier<AlignmentUtils.ProcessorSelection> processorSelectionSupplier) {

    // Step 2: Get the StrategyManager to dynamically select alignment strat
    StrategyManager strategyManager = drive.getStrategyManager();

    // Create an initialization command that resets the controller using current
    // sensor values.
    Command initCommand =
        new InstantCommand(
            () -> {
              AlignmentContext initContext =
                  new AlignmentContext(
                      robotPoseSupplier.get(),
                      haveCoralSupplier.get(),
                      haveAlgaeSupplier.get(),
                      elevatorHeightInchesSupplier.getAsDouble(),
                      climberDeployedSupplier.get(),
                      reefFaceSelectionSupplier.get(),
                      coralStationSelectionSupplier.get(),
                      cageSelectionSupplier.get(),
                      processorSelectionSupplier.get());
              strategyManager.initialControllerReset(initContext);
            },
            drive);

    // Create the main drive command as before.
    Command driveCommand =
        Commands.run(
            () -> {
              // Step 1: Get joystick inputs
              double rawX = xSupplier.getAsDouble();
              double rawY = ySupplier.getAsDouble();
              double rawOmega = omegaSupplier.getAsDouble();

              // Convert joystick inputs to a field-relative translation
              Translation2d rawTranslationInput = getLinearVelocityFromJoysticks(rawX, rawY);
              double rawRotationInput = MathUtil.applyDeadband(rawOmega, DEADBAND);
              rawRotationInput =
                  Math.copySign(
                      rawRotationInput * rawRotationInput,
                      rawRotationInput); // Squaring for finer control

              // Step 2: Construct the AlignmentContext
              AlignmentContext context =
                  new AlignmentContext(
                      robotPoseSupplier.get(),
                      haveCoralSupplier.get(),
                      haveAlgaeSupplier.get(),
                      elevatorHeightInchesSupplier.getAsDouble(),
                      climberDeployedSupplier.get(),
                      reefFaceSelectionSupplier.get(),
                      coralStationSelectionSupplier.get(),
                      cageSelectionSupplier.get(),
                      processorSelectionSupplier.get());

              // Step 3: Update alignment strategy
              strategyManager.updateStrategyForCycle(context);

              // Step 4: Apply corrections
              double adjustedRotationInput =
                  strategyManager.getRotationalCorrection(context, rawRotationInput);
              Translation2d adjustedTranslationInput =
                  strategyManager.getTranslationalCorrection(context, rawTranslationInput);

              // Step 5: Send to drive system
              sendSpeedsToDrive(drive, adjustedTranslationInput, adjustedRotationInput);
            },
            drive);

    // Sequence the initialization command before the drive command.
    return Commands.sequence(initCommand, driveCommand);
  }

  /**
   * Sends the computed chassis speeds to the drive subsystem.
   *
   * @param drive The drive subsystem.
   * @param translationInput The field-relative linear velocity.
   * @param rotationInput The angular velocity.
   */
  private static void sendSpeedsToDrive(
      Drive drive, Translation2d translationInput, double rotationInput) {

    double linearSpeedScalar = 1.0;
    double rotationSpeedScalar = 1.0;
    if (drive.getElevatorHeightLimitsSpeed()) {
      linearSpeedScalar = calculateSpeedMultiplier(RobotStatus.elevatorHeightInches());
      rotationSpeedScalar = calculateRotationSpeedMultiplier(RobotStatus.elevatorHeightInches());
    }

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            translationInput.getX() * drive.getMaxLinearSpeedMetersPerSec() * linearSpeedScalar,
            translationInput.getY() * drive.getMaxLinearSpeedMetersPerSec() * linearSpeedScalar,
            rotationInput * drive.getMaxAngularSpeedRadPerSec() * rotationSpeedScalar);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  private static double calculateSpeedMultiplier(double elevatorHeightInches) {
    // Convert the elevator height from inches to meters.
    double elevatorHeightMeters = elevatorHeightInches * 0.0254;

    // Define thresholds (in meters):
    // 25 inches = 25 * 0.0254 ≈ 0.635 m (full speed)
    // 50 inches = 50 * 0.0254 ≈ 1.27 m (minimum speed)
    final double fullSpeedThreshold = 25 * 0.0254; // ≈ 0.635 meters
    final double reducedSpeedThreshold = 50 * 0.0254; // ≈ 1.27 meters
    final double minMultiplier = 0.2; // Minimum multiplier at maximum height

    if (elevatorHeightMeters <= fullSpeedThreshold) {
      return 1.0;
    } else if (elevatorHeightMeters >= reducedSpeedThreshold) {
      return minMultiplier;
    } else {
      // Linearly interpolate between full speed (1.0) and minimum speed
      // (minMultiplier)
      double fraction =
          (elevatorHeightMeters - fullSpeedThreshold)
              / (reducedSpeedThreshold - fullSpeedThreshold);
      return 1.0 - fraction * (1.0 - minMultiplier);
    }
  }

  private static double calculateRotationSpeedMultiplier(double elevatorHeightInches) {
    // Convert the elevator height from inches to meters.
    double elevatorHeightMeters = elevatorHeightInches * 0.0254;

    // Define thresholds (in meters) for rotational speed:
    // Below 25 inches (~0.635 m), full rotational speed is allowed.
    // Above 50 inches (~1.27 m), use the minimum rotational speed.
    final double fullSpeedThreshold = 25 * 0.0254; // ≈ 0.635 meters
    final double reducedSpeedThreshold = 50 * 0.0254; // ≈ 1.27 meters
    final double minRotationMultiplier = 0.5; // Adjust as needed

    if (elevatorHeightMeters <= fullSpeedThreshold) {
      return 1.0;
    } else if (elevatorHeightMeters >= reducedSpeedThreshold) {
      return minRotationMultiplier;
    } else {
      double fraction =
          (elevatorHeightMeters - fullSpeedThreshold)
              / (reducedSpeedThreshold - fullSpeedThreshold);
      return 1.0 - fraction * (1.0 - minRotationMultiplier);
    }
  }
}
