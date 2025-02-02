package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSubsystem;

public class CoralSystemCommands {

  private CoralSystemCommands() {}

  /**
   * Set the elevator position using a value from SmartDashboard. This command requires the
   * CoralSubsystem and calls the elevator component within it.
   */
  public static Command setElevatorPositionFromDashboard(CoralSubsystem coralSubsystem) {
    return Commands.runOnce(
        () -> {
          double goal = SmartDashboard.getNumber("Elevator Goal", 0);
          coralSubsystem.getElevator().setPosition(goal);
        },
        coralSubsystem // The requirement is the entire CoralSubsystem.
        );
  }

  /**
   * Set the elevator position to a specific requested value. The command requires the
   * CoralSubsystem.
   */
  public static Command setElevatorPosition(
      CoralSubsystem coralSubsystem, Double requestedPosition) {
    return Commands.runOnce(
        () -> coralSubsystem.getElevator().setPosition(requestedPosition), coralSubsystem);
  }

  /**
   * Set the arm angle using a value from SmartDashboard. This command requires the CoralSubsystem
   * and calls the arm component within it.
   */
  public static Command setArmAngleFromDashboard(CoralSubsystem coralSubsystem) {
    return Commands.runOnce(
        () -> {
          double goal = SmartDashboard.getNumber("Arm Angle Goal", 0);
          coralSubsystem.getArm().setAngle(goal);
        },
        coralSubsystem);
  }

  /** Set the arm angle to a specific requested value. The command requires the CoralSubsystem. */
  public static Command setArmAngle(CoralSubsystem coralSubsystem, Double requestedAngle) {
    return Commands.runOnce(() -> coralSubsystem.getArm().setAngle(requestedAngle), coralSubsystem);
  }
}
