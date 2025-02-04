package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSystem;

public class ElevatorCommands {

  private ElevatorCommands() {}

  /** Set the Elevator Position */
  public static Command setElevatorPositionFromDashboard(CoralSystem coralSystem) {
    return Commands.runOnce(
        () -> {
          coralSystem.getElevator().setPositionInches(SmartDashboard.getNumber("Elevator Goal", 0));
        },
        coralSystem);
  }

  public static Command setElevatorPosition(CoralSystem coralSystem, Double requestedPosition) {
    return Commands.runOnce(
        () -> {
          coralSystem.getElevator().setPosition(requestedPosition);
        },
        coralSystem);
  }
}
