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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  // Time tracking for limiting vision updates
  private double lastUpdateTime = 0.0;
  // Update interval in seconds (e.g., 50 ms)
  private static final double UPDATE_INTERVAL = 0.05;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The transform from the robot to the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize the vision simulation (only once)
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Configure the simulated camera properties.
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(70));

    // Create the PhotonCameraSim instance.
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    // Only update the vision simulation if the interval has elapsed.
    if (now - lastUpdateTime >= UPDATE_INTERVAL) {
      visionSim.update(poseSupplier.get());
      lastUpdateTime = now;
    }
    // Call the superclass update to propagate any additional behavior.
    super.updateInputs(inputs);
  }
}
