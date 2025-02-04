package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import lombok.Getter;

public class CoralSystem extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;

  public CoralSystem(Elevator elevator, Arm arm) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
    this.elevator = elevator;
    this.arm = arm;
    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this));
  }

  @Override
  public void periodic() {
    this.elevator.periodic();
    this.arm.periodic();
  }
}
