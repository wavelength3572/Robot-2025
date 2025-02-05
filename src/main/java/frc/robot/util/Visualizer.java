package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Visualizer {

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Double> elevatorHeightSupplier;
  private final Supplier<Double> armAngleSupplier;
  private final Supplier<Boolean> isCoralInRobotSupplier;

  public Visualizer(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> elevatorHeightSupplier,
      Supplier<Double> armAngleSupplier,
      Supplier<Boolean> isCoralInRobotSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.elevatorHeightSupplier = elevatorHeightSupplier;
    this.armAngleSupplier = armAngleSupplier;
    this.isCoralInRobotSupplier = isCoralInRobotSupplier;
  }

  /** Updates 3D visualization for the robot's components and game pieces */
  public void updateVisualization() {
    Pose2d robotPose2d = robotPoseSupplier.get();
    double elevatorHeight = elevatorHeightSupplier.get();
    double armAngleDegrees = armAngleSupplier.get();

    // 🔹 Compute dynamic elevator pose
    Pose3d elevatorPose = getElevator3DPose(elevatorHeight);

    // 🔹 Compute dynamic arm pose
    Pose3d armPose = getArm3DPose(elevatorHeight, armAngleDegrees);

    // 🔹 Apply calibration offset (-90 degrees) to align arm visualization
    Pose3d armCalibratedPose =
        armPose.plus(new Transform3d(0, 0, 0, new Rotation3d(0, Units.degreesToRadians(-90), 0)));

    // ✅ Log robot components (elevator + calibrated arm)
    Logger.recordOutput("FinalComponentPoses", new Pose3d[] {elevatorPose, armCalibratedPose});

    if (isCoralInRobotSupplier.get()) {
      // ✅ Move coral relative to the robot's 2D pose, applying zeroed position & rotations
      Pose3d coralPose = attachCoralToRobot(robotPose2d, armCalibratedPose);
      Logger.recordOutput("Coral", coralPose);
    } else {
      // ✅ Restore coral to its original staged position
      Pose3d stagedCoralPose = new Pose3d(0.5, 1.0, -0.5, new Rotation3d(0, 0, 0)); // Match JSON
    //   Logger.recordOutput("Coral", new Pose3d());
    }
  }

  /** Computes and returns the 3D pose of the elevator for visualization */
  private Pose3d getElevator3DPose(double elevatorOffset) {
    return new Pose3d(0, -0.0908, elevatorOffset + 0.24, new Rotation3d(0, 0, 0));
  }

  /** Computes and returns the 3D pose of the arm for visualization */
  private Pose3d getArm3DPose(double elevatorHeight, double armAngleDegrees) {
    double armAngleRadians = Units.degreesToRadians(armAngleDegrees); // ✅ Convert inside the method
    return new Pose3d(
        0,
        -0.190,
        elevatorHeight + 0.262,
        new Rotation3d(0, armAngleRadians, Units.degreesToRadians(180)));
  }

  /**
   * Calculates and returns the coral's pose when attached to the robot, applying zeroed rotations &
   * position
   */
  private Pose3d attachCoralToRobot(Pose2d robotPose2d, Pose3d armPose) {
    return new Pose3d(
        robotPose2d.getX() + armPose.getX()+.205, // 🔹 Offset coral by robot position
        robotPose2d.getY() + armPose.getY()+.02,
        armPose.getZ()+.31, // Keep arm height
        new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()) // Apply robot rotation
        );
  }
}
