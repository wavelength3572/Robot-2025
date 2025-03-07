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
import frc.robot.subsystems.algae.AlgaeConstants;
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
  private final Supplier<Boolean> haveCoralSupplier;
  private final Supplier<Boolean> haveAlgaeSupplier;

  private final Supplier<Double> algaeDeployPositionSupplier;

  private LoggedMechanism2d coralSystem2D;
  private LoggedMechanismRoot2d coralRoot;
  private LoggedMechanismLigament2d m_elevator;

  private LoggedMechanism2d algaeSystem2D;
  private LoggedMechanismRoot2d algaeRoot;
  private LoggedMechanismLigament2d algaeDeployArm;
  private LoggedMechanismLigament2d algaeCaptureMotor;

  private double visualDeployArmOffset = -90;

  public Visualizer(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> elevatorHeightSupplier,
      Supplier<Double> armAngleSupplier,
      Supplier<Boolean> haveCoralSupplier,
      Supplier<Boolean> haveAlgaeSupplier,
      Supplier<Double> algaeDeployPositionSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.elevatorHeightSupplier = elevatorHeightSupplier;
    this.armAngleSupplier = armAngleSupplier;
    this.haveCoralSupplier = haveCoralSupplier;
    this.haveAlgaeSupplier = haveAlgaeSupplier;
    this.algaeDeployPositionSupplier = algaeDeployPositionSupplier;
    initializeCoral2DVisualization();
    initializeAlgae2DVisualization();
  }

  /** Initializes the 2D visualization for the Coral system */
  private void initializeCoral2DVisualization() {
    coralSystem2D = new LoggedMechanism2d(.8382, 2.0);
    coralRoot = coralSystem2D.getRoot("Coral Base", 0.51, 0.0);

    m_elevator =
        coralRoot.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                ElevatorConstants.kGroundToElevator,
                90,
                2,
                new Color8Bit(Color.kBlue)));
  }

  private void initializeAlgae2DVisualization() {
    algaeSystem2D = new LoggedMechanism2d(1.0, 1.0); // Separate system

    // Offset Algae Base to the side of the robot
    algaeRoot = algaeSystem2D.getRoot("Algae Base", 1.0, 0.1); // X = 1.0 moves it to the right side

    // Algae deploy arm visualization (now offset correctly)
    algaeDeployArm =
        algaeRoot.append(
            new LoggedMechanismLigament2d(
                "Algae Deploy Arm",
                0.3, // Arm length
                visualDeployArmOffset, // Initial angle
                3,
                new Color8Bit(Color.kGreen)));

    // Algae capture motor (indicator for intake/outtake)
    algaeCaptureMotor =
        algaeDeployArm.append(
            new LoggedMechanismLigament2d(
                "Algae Capture Motor", 0.1, 0, 4, new Color8Bit(Color.kRed)));
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

    double algaeAngleDegrees = algaeDeployPositionSupplier.get();

    // 🔹 Compute dynamic arm pose
    Pose3d algaePose = getAlgae3DPose(algaeAngleDegrees);

    // 🔹 Apply calibration offset (-90 degrees) to align arm visualization
    Pose3d algaeCalibratedPose =
        algaePose.plus(new Transform3d(0, 0, 0, new Rotation3d(Math.toRadians(103), 0, 0)));

    // ✅ Log robot components (elevator + calibrated arm + algae collector)
    Logger.recordOutput(
        "Visualizer/FinalComponentPoses",
        new Pose3d[] {elevatorPose, armCalibratedPose, algaeCalibratedPose});

    if (haveCoralSupplier.get()) {
      // ✅ Move coral relative to the robot's 2D pose, applying zeroed position &
      // rotations
      Pose3d coralPose = attachCoralToRobot(robotPose2d, armCalibratedPose);
      Logger.recordOutput("Visualizer/HaveCoralRobotPose", coralPose);
    } else {
      Logger.recordOutput("Visualizer/HaveCoralRobotPose", new Pose3d());
    }

    // 🔹 Log scored corals on the reef poles
    logScoredCorals();
    Logger.recordOutput("Visualizer/StagedAlgae", FieldConstants.getAllStagedAlgaePositions());
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

  /** Computes and returns the 3D pose of the arm for visualization */
  private Pose3d getAlgae3DPose(double algaeAngleDegrees) {
    double algaeAngleRadians =
        Units.degreesToRadians(algaeAngleDegrees); // ✅ Convert inside the method
    return new Pose3d(-0.240, 0.212, 0.191, new Rotation3d(-algaeAngleRadians, 0, 0));
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
        new Transform3d(new Translation3d(-.178, 0.0, 0.42), new Rotation3d(0, 0, 0));
    // -.022 might be right for the y

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

    double algaeDeployPosition = algaeDeployPositionSupplier.get();
    // double algaeCaptureSpeed = algaeCaptureSpeedSupplier.get();

    // Update algae arm position

    double gearRatio = AlgaeConstants.kAlgaeDeployGearing;
    double rotations = algaeDeployPosition;
    double angleDegrees = ((rotations / gearRatio) * 360.0) + visualDeployArmOffset;

    algaeDeployArm.setAngle(-angleDegrees);

    // Change color of capture motor based on speed (indicating intake/outtake)
    // if (algaeCaptureSpeed > 0.1) {
    //   algaeCaptureMotor.setColor(new Color8Bit(Color.kGreen)); // Intake
    // } else if (algaeCaptureSpeed < -0.1) {
    //   algaeCaptureMotor.setColor(new Color8Bit(Color.kRed)); // Outtake
    // } else {
    //   algaeCaptureMotor.setColor(new Color8Bit(Color.kGray)); // Idle
    // }

    // ✅ Log the separate 2D visualizations
    Logger.recordOutput("Visualizer/Coral System 2D", coralSystem2D);
    Logger.recordOutput("Visualizer/Algae System 2D", algaeSystem2D);
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

    Logger.recordOutput("Visualizer/ScoredCorals", scoredCoralPoses.toArray(new Pose3d[0]));
  }
}
