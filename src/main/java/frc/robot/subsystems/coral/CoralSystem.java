package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class CoralSystem extends SubsystemBase {

  private Elevator elevator;

  private final CoralSystemPresetChooser coralSystemPresetChooser;

  public Elevator getElevator() {
    return elevator;
  }

  private Arm arm;

  public Arm getArm() {
    return arm;
  }

  public CoralSystem(Elevator elevator, Arm arm) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
    this.elevator = elevator;
    this.arm = arm;
  }

  @Override
  public void periodic() {
    this.elevator.periodic();
    this.arm.periodic();
  }
}
