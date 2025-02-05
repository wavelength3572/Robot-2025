package frc.robot.subsystems.coral;

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
  }
}
