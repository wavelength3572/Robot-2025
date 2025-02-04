package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSystemPresetChooser;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.coral.CoralSystemPresets.CoralState;

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
          double goal = SmartDashboard.getNumber("Arm Goal", 0);
          coralSubsystem.getArm().setAngleDegrees(goal);
        },
        coralSubsystem);
  }

  /** Set the arm angle to a specific requested value. The command requires the CoralSubsystem. */
  public static Command setArmAngle(CoralSubsystem coralSubsystem, Double requestedAngle) {
    return Commands.runOnce(
        () -> coralSubsystem.getArm().setAngleDegrees(requestedAngle), coralSubsystem);
  }

  public static Command stow(CoralSubsystem coralSubsystem) {
    return Commands.runOnce(
        () -> {
          coralSubsystem.getElevator().setPosition(CoralSystemPresets.STOW.getElevatorHeight());
          coralSubsystem.getArm().setAngleDegrees(CoralSystemPresets.STOW.getArmAngle());
        },
        coralSubsystem);
  }

  public static Command getCoralSelectedPresetFromSmartDashboardCommand(
      CoralSubsystem coralSubsystem, CoralSystemPresetChooser presetChooser) {
    return Commands.runOnce(
        () -> {
          CoralSystemPresets preset = presetChooser.getSelected();
          coralSubsystem.getElevator().setPosition(preset.getElevatorHeight());
          coralSubsystem.getArm().setAngleDegrees(preset.getArmAngle());
        },
        coralSubsystem);
  }

  /**
   * Creates a command that holds the current arm position. This is useful as a default command to
   * prevent sudden movements.
   */
  public static Command holdArmAngle(CoralSubsystem coralSubsystem) {
    return Commands.runOnce(coralSubsystem.getArm()::holdArmAngle, coralSubsystem);
  }

  public static Command moveToState(CoralSubsystem coralSubsystem, CoralState targetState) {
    return Commands.runOnce(
        () -> {
          // Use setState to ensure the FSM updates correctly
          coralSubsystem.getStateMachine().setState(targetState);
        },
        coralSubsystem);
  }

  /** Moves the system to the pickup position. */
  public static Command prepareToPickupCoral(CoralSubsystem coralSubsystem) {
    return moveToState(coralSubsystem, CoralState.PICKUP);
  }

  /** Moves the system to the currently selected scoring position. */
  public static Command prepareToScoreCoral(CoralSubsystem coralSubsystem) {
    return Commands.runOnce(
        () -> {
          CoralState selectedLevel = coralSubsystem.getSelectedScoringLevel();
          moveToState(coralSubsystem, selectedLevel).schedule();
        });
  }

  /** Moves the system to the default stow position (STOW_LOW). */
  public static Command prepareToStow(CoralSubsystem coralSubsystem) {
    return moveToState(coralSubsystem, CoralState.STOW_LOW);
  }

  /** Moves the system to the front algae dislodge position. */
  public static Command prepareToAlgaeDislodge(CoralSubsystem coralSubsystem) {
    return moveToState(coralSubsystem, CoralState.FRONT_ALGAE_DISLODGE);
  }

  /** Releases the coral from the intake mechanism and stops after a delay. */
  public static Command releaseCoral(CoralSubsystem coralSubsystem) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              CoralState currentState = coralSubsystem.getStateMachine().getCurrentState();
              if (currentState == CoralState.L1_SCORE
                  || currentState == CoralState.L2_SCORE
                  || currentState == CoralState.L3_SCORE
                  || currentState == CoralState.L4_SCORE) {

                coralSubsystem.getEndEffector().runOpenLoop(-1.0); // Eject coral
                System.out.println("🦞 Coral Released!");
              }
            },
            coralSubsystem),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> coralSubsystem.getEndEffector().stop(), coralSubsystem));
  }
  
  
  public static Command goToLevelOne(CoralSubsystem coralSubsystem, CoralSystemPresetChooser presetChooser) {
  return Commands.runOnce(
      () -> {
        CoralSystemPresets preset = presetChooser.getSelected();
        coralSubsystem.getElevator().setPosition(preset.getElevatorHeight());
        coralSubsystem.getArm().setAngleDegrees(preset.getArmAngle());
      },
      coralSubsystem);
}

}
