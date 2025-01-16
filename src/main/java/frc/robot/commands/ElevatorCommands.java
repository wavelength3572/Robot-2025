package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  private ElevatorCommands() {}

  /** Set the Elevator Position */
  public static Command setElevatorPositionFromDashboard(Elevator elevator) {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(SmartDashboard.getNumber("Elevator Goal", 0));
        },
        elevator);
  }

  public static Command setElevatorPosition(Elevator elevator, Double requestedPosition) {
    return Commands.runOnce(
        () -> {
          elevator.setPosition(requestedPosition);
        },
        elevator);
  }
}
