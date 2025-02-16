package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.FieldConstants;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Visualizer {

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Double> elevatorHeightSupplier;
  private final Supplier<Double> armAngleSupplier;
  private final Supplier<Boolean> isCoralInRobotSupplier;

  private LoggedMechanism2d coralSystem2D;
  private LoggedMechanismRoot2d root;
  private LoggedMechanismLigament2d m_elevator;

  public Visualizer(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> elevatorHeightSupplier,
      Supplier<Double> armAngleSupplier,
      Supplier<Boolean> isCoralInRobotSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.elevatorHeightSupplier = elevatorHeightSupplier;
    this.armAngleSupplier = armAngleSupplier;
    this.isCoralInRobotSupplier = isCoralInRobotSupplier;
    initialize2DVisualization();
  }

  /** Initializes the 2D visualization for the robot's components */
  public void initialize2DVisualization() {
    coralSystem2D = new LoggedMechanism2d(.8382, 2.0);
    root = coralSystem2D.getRoot("Base", 0.51, 0.0);
    m_elevator =
        root.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                ElevatorConstants.kGroundToElevator,
                90,
                2,
                new Color8Bit(Color.kBlue)));
  }

  /** Updates 3D visualization for the robot's components and game pieces */
  public void update3DVisualization() {
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
      // ✅ Move coral relative to the robot's 2D pose, applying zeroed position &
      // rotations
      Pose3d coralPose = attachCoralToRobot(robotPose2d, armCalibratedPose);
      Logger.recordOutput("Coral", coralPose);
    } else {
      Logger.recordOutput("Coral", new Pose3d());
    }

    // 🔹 Log scored corals on the reef poles
    logScoredCorals();
    Logger.recordOutput("Algae/StagedAlgae", FieldConstants.getAllStagedAlgaePositions());
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

  private Pose3d attachCoralToRobot(Pose2d robotPose2d, Pose3d armPoseInRobotFrame) {
    // (1) Convert the 2D robot pose (x, y, yaw) to a full 3D Pose3d.
    Rotation3d robotYaw = new Rotation3d(0, 0, robotPose2d.getRotation().getRadians());
    Pose3d robotPose3d = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, robotYaw);

    // (2) Build a Transform3d that represents how to go from the robot frame to the
    // arm frame.
    // If armPoseInRobotFrame is the arm’s Pose3d in robot coords, then
    // robotToArm = new Transform3d(/*from*/ origin, /*to*/ armPoseInRobotFrame)
    Transform3d robotToArm = new Transform3d(new Pose3d(), armPoseInRobotFrame);

    // (3) Build the final offset from the arm to the coral (local translation &
    // rotation).
    // This is the "fine tuning" that says, "the coral is 0.202 m forward from the
    // pivot, etc."
    Transform3d armToCoral =
        new Transform3d(new Translation3d(-.21, -.0155, 0.3135), new Rotation3d(0, 0, 0));

    // (4) Compose them:
    // fieldToCoral = fieldToRobot * robotToArm * armToCoral
    Pose3d coralInField =
        robotPose3d
            .transformBy(robotToArm) // robot -> arm
            .transformBy(armToCoral); // arm -> coral

    return coralInField;
  }

  /** Updates the 2D visualization for the elevator */
  public void update2DVisualization() {
    double elevatorHeight = elevatorHeightSupplier.get();
    m_elevator.setLength(ElevatorConstants.kGroundToElevator + elevatorHeight);

    Logger.recordOutput("Coral System 2D", coralSystem2D);
  }

  private void logScoredCorals() {
    List<Pose3d> scoredCoralPoses = new ArrayList<>();

    // Iterate over the coralMapping and collect poses that have been scored.
    for (Map.Entry<FieldConstants.CoralKey, FieldConstants.CoralLocation> entry :
        FieldConstants.coralMapping.entrySet()) {
      if (entry.getValue().scored) {
        scoredCoralPoses.add(entry.getValue().pose);
      }
    }

    Logger.recordOutput("ScoredCorals", scoredCoralPoses.toArray(new Pose3d[0]));
  }
}
