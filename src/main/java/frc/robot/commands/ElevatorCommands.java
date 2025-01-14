package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  private ElevatorCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command setElevatorPosition(Elevator elevator, Double requestedPosition) {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(requestedPosition);
        },
        elevator);
  }
}
