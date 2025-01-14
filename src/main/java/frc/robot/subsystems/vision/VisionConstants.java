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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public final class VisionConstants {
  // AprilTag layout
  // public static AprilTagFieldLayout aprilTagLayout =
  // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static AprilTagFieldLayout aprilTagLayout = null;

  static {
    try {
      aprilTagLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json"));
    } catch (IOException e) {
      aprilTagLayout = null;
      // e.printStackTrace();
      // throw new Error("2025-reefscape.json does not exists in the deploy
      // directory");
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";
  public static String camera2Name = "camera_2";

  // Robot to camera transforms
  public static Transform3d robotToCamera0 =
      new Transform3d(
          0.2517648,
          -0.2517648,
          0.2102358,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(0).getRadians(),
              Rotation2d.fromDegrees(-90).getRadians()));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.2517648,
          0.2517648,
          0.2102358,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-30).getRadians(),
              Rotation2d.fromDegrees(45).getRadians()));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.2517648,
          -0.2517648,
          0.2102358,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(0).getRadians(),
              Rotation2d.fromDegrees(180).getRadians()));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  private VisionConstants() {}
}
