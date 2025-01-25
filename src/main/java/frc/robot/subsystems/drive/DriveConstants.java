// Copyright 2021-2024 FRC 6328
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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(21.25);
  public static final double wheelBase = Units.inchesToMeters(21.25);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  // Rotation2d(2.520);
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.636);
  // Rotation2d(-1.132);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(2.0187);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-2.1785);
  // Rotation2d(1.621);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.5226);

  // Device CAN IDs
  public static final int pigeonCanId = 19;

  public static final int frontLeftDriveCanId = 5;
  public static final int backLeftDriveCanId = 8;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 6;

  public static final int frontLeftTurnCanId = 9;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 7;
  public static final int backRightTurnCanId = 2;

  public static final int frontLeftCANCoderId = 13;
  public static final int backLeftCANCoderId = 12;
  public static final int frontRightCANCoderId = 11;
  public static final int backRightCANCoderId = 10;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // Gear ratios for
  // SDS MK4i L2
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  public static final double maxSpeedMetersPerSec =
      5676.0 // Free Speed of NEO from website
          / 60.0
          / driveMotorReduction
          * 2.0
          * Math.PI
          * wheelRadiusMeters;

  // Drive encoder configuration
  public static final boolean driveInverted = false;
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.11468;
  public static final double driveKv = 0.13085;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.04307;
  public static final double driveSimKv = 0.21126;

  public static final double simStartX = 2;
  public static final double simStartY = 7;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150.0 / 7.0; // SDS MK4i L2
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel
  // Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 54.43;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.0; // From
  // https://www.vexrobotics.com/colsonperforma.html#attr-vex_resources
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
