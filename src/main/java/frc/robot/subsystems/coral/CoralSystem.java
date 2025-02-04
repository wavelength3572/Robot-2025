package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import lombok.Getter;

public class CoralSystem extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  public CoralSystem(Elevator elevator, Arm arm, Intake intake) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
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
  }
}
