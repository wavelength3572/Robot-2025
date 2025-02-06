package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;
  @AutoLogOutput @Getter public boolean coralInRobot;

  @Getter
  private CoralSystemPresets targetCoralPreset =
      CoralSystemPresets.STARTUP; // Default startup position

  @Getter
  private CoralSystemPresets currentCoralPreset =
      CoralSystemPresets.STARTUP; // Tracks last reached preset

  public CoralSystem(Elevator elevator, Arm arm, Intake intake) {
    coralSystemPresetChooser = new CoralSystemPresetChooser(this);
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;

    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this));
  }

  @Override
  public void periodic() {
    this.elevator.periodic();
    this.arm.periodic();
    this.intake.periodic();

    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    coralSystemPresetChooser.checkAndUpdate();

    Pose3d elevatorDynamicPose = this.elevator.getElevator3DPose();
    Pose3d armDynamicPose = this.arm.getArm3DPose(elevator::getHeightInMeters);
    Pose3d armCalibratedPose =
        armDynamicPose.plus(
            new Transform3d(0, 0, 0, new Rotation3d(0, Units.degreesToRadians(-90), 0)));
    Logger.recordOutput(
        "FinalComponentPoses", new Pose3d[] {elevatorDynamicPose, armCalibratedPose});

    // Pose3d coralPose;
    // Pose3d stagedCoralPose = new Pose3d();
    // Pose3d robotCoralPose = new Pose3d(1.5, 1.5, 1.5, new Rotation3d(0, 0, 0)); // Fake robot
    // position

    // if (isCoralInRobot()) {
    //   // Coral is part of the robot
    //   // coralPose = attachCoralToRobot(armCalibratedPose);
    //   // Logger.recordOutput("FinalComponentPoses", new Pose3d[] {elevatorDynamicPose,
    // armCalibratedPose, coralPose});
    //   Logger.recordOutput("FieldObjects/Coral0", robotCoralPose);
    // } else {
    //   // coralPose = getNeutralCoralPose();
    //   // Logger.recordOutput("FinalComponentPoses", new Pose3d[] {elevatorDynamicPose,
    // armCalibratedPose, coralPose});
    //   Logger.recordOutput("FieldObjects/Coral0", stagedCoralPose);
    // }
  }

  public void setCoralInRobot(Boolean coralInRobot) {
    this.coralInRobot = coralInRobot;
  }

  public boolean isAtGoal() {
    boolean atGoal = elevator.isAtGoal() && arm.isAtGoal();
    if (atGoal) currentCoralPreset = targetCoralPreset;
    return atGoal;
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    this.targetCoralPreset = preset;
    elevator.setTargetPreset(preset);
    arm.setTargetPreset(preset);
  }

  /** Calculates and returns the coral's pose when attached to the robot */
  private Pose3d attachCoralToRobot(Pose3d armPose) {
    return new Pose3d(
        armPose.getX(), // Adjust forward/backward offset
        armPose.getY(), // Keep Y same
        armPose.getZ(), // Adjust height
        armPose.getRotation() // Rotate with arm
        );
  }

  /** Returns a "neutral" pose when coral is not attached to the robot */
  private Pose3d getNeutralCoralPose() {
    return new Pose3d(0, 0, -10, new Rotation3d(0, 0, 0)); // Place far below field level
  }
}
