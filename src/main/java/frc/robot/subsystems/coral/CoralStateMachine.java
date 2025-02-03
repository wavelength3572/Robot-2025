package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.CoralSystemPresets.CoralState;
import org.littletonrobotics.junction.Logger;

public class CoralStateMachine {
  private CoralState currentState = CoralState.STOW;
  private final CoralSubsystem coralSubsystem;

  public CoralStateMachine(CoralSubsystem coralSubsystem) {
    this.coralSubsystem = coralSubsystem;
  }

  /** Updates state transitions based on movement completion. */
  public void update() {
    boolean movementComplete = isMovementComplete();

    // Log FSM state and motion status
    Logger.recordOutput("Coral/FSM State", currentState.name());
    Logger.recordOutput("Coral/Motion Complete", movementComplete);

    // If movement is done, print a debug message
    if (movementComplete && isPrepareState(currentState)) {
      CoralState finalState = getFinalState(currentState);
      Logger.recordOutput("Coral/FSM Transitioning To", finalState.name());
      setState(finalState);
    }
  }

  /** Moves to a new state (any transition allowed for testing). */
  public void setState(CoralState newState) {
    Logger.recordOutput("Coral/FSM Transition", "From " + currentState + " to " + newState);
    currentState = newState;
    applyPreset(newState);
  }

  /** Moves the elevator and arm based on the state. */
  private void applyPreset(CoralState state) {
    CoralSystemPresets preset = getPresetForState(state);
    if (preset == null) {
      Logger.recordOutput("Coral/FSM ERROR", "❌ No preset found for state " + state);
      return;
    }

    // Move the elevator and arm
    coralSubsystem.getElevator().setPosition(preset.getElevatorHeight());
    coralSubsystem.getArm().setAngleDegrees(preset.getArmAngle());

    // Log movement
    Logger.recordOutput("Coral/Elevator Target", preset.getElevatorHeight());
    Logger.recordOutput("Coral/Arm Target", preset.getArmAngle());
  }

  /** Finds the correct preset for a given CoralState */
  private CoralSystemPresets getPresetForState(CoralState state) {
    for (CoralSystemPresets preset : CoralSystemPresets.values()) {
      if (preset.getState() == state) {
        return preset;
      }
    }
    return null;
  }

  /** Returns whether the arm and elevator have reached their goals. */
  private boolean isMovementComplete() {
    boolean elevatorAtGoal = coralSubsystem.getElevator().isAtGoal();
    boolean armAtGoal = coralSubsystem.getArm().isAtGoal();
    boolean complete = elevatorAtGoal && armAtGoal;

    Logger.recordOutput("Coral/Elevator isAtGoal", elevatorAtGoal);
    Logger.recordOutput("Coral/Arm isAtGoal", armAtGoal);

    return complete;
  }

  /** Determines if a state is a PREPARE_ state. */
  private boolean isPrepareState(CoralState state) {
    return state.name().startsWith("PREPARE_");
  }

  /** Converts a PREPARE_ state into its final state. */
  private CoralState getFinalState(CoralState prepareState) {
    return CoralState.valueOf(prepareState.name().replace("PREPARE_", ""));
  }

  /** Returns the current state. */
  public CoralState getCurrentState() {
    return currentState;
  }
}
