package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.CoralSystemPresets.CoralState;
import java.util.EnumSet;
import org.littletonrobotics.junction.Logger;

public class CoralStateMachine {
  private CoralState currentState = CoralState.STOW;
  private final CoralSubsystem coralSubsystem;
  private long stateStartTime = System.currentTimeMillis();

  public CoralStateMachine(CoralSubsystem coralSubsystem) {
    this.coralSubsystem = coralSubsystem;
  }

  /** Updates state transitions based on movement completion. */
  public void update() {
    boolean movementComplete = isMovementComplete();
    long elapsedTime = System.currentTimeMillis() - stateStartTime;

    // Log FSM state and motion status
    Logger.recordOutput("Coral/FSM State", currentState.name());
    Logger.recordOutput("Coral/FSM Movement Complete", movementComplete);
    Logger.recordOutput("Coral/FSM Time in State (ms)", elapsedTime);

    if (isPrepareState(currentState)) {
      if (movementComplete) {
        // Log state transition
        CoralState finalState = getFinalState(currentState);
        Logger.recordOutput("Coral/FSM Transitioning To", finalState.name());
        setState(finalState);
      } else if (elapsedTime > 3000) { // 3-second timeout
        Logger.recordOutput("Coral/FSM Timeout", true);
        setState(getFinalState(currentState));
      }
    }
  }

  /** Moves to a new valid state. */
  public void setState(CoralState newState) {
    if (!EnumSet.allOf(CoralState.class).contains(newState)) {
      Logger.recordOutput("Coral/FSM ERROR", "Invalid state: " + newState);
      return;
    }

    if (isFinalState(newState)) {
      Logger.recordOutput(
          "Coral/FSM ERROR", "Can't directly transition to final state: " + newState);
      return;
    }

    if (isValidTransition(newState)) {
      Logger.recordOutput("Coral/FSM Transition", "From " + currentState + " to " + newState);
      currentState = newState;
      applyPreset(newState);

      // Reset state timer
      stateStartTime = System.currentTimeMillis();

      if (isPrepareState(newState) && isMovementComplete()) {
        CoralState finalState = getFinalState(newState);
        Logger.recordOutput("Coral/FSM Instant Transition", finalState.name());
        currentState = finalState;
      }
    } else {
      Logger.recordOutput(
          "Coral/FSM Invalid Transition", "From " + currentState + " to " + newState);
    }
  }

  /** Checks if the requested transition is allowed. */
  private boolean isValidTransition(CoralState newState) {
    switch (currentState) {
      case STOW:
      case SAFE_CARRIAGE_POSITION:
        return newState == CoralState.PREPARE_SAFE_CARRIAGE_POSITION
            || newState == CoralState.PREPARE_CORAL_STATION_PICKUP
            || newState == CoralState.PREPARE_L1_SCORE
            || newState == CoralState.PREPARE_L2_SCORE
            || newState == CoralState.PREPARE_L3_SCORE
            || newState == CoralState.PREPARE_L4_SCORE;
      case PICKUP:
        return newState == CoralState.PREPARE_STOW_LOW;
      case STOW_LOW:
        return newState == CoralState.PREPARE_CORAL_STATION_PICKUP
            || newState == CoralState.PREPARE_L1_SCORE
            || newState == CoralState.PREPARE_L2_SCORE
            || newState == CoralState.PREPARE_L3_SCORE
            || newState == CoralState.PREPARE_L4_SCORE
            || newState == CoralState.PREPARE_FRONT_ALGAE_DISLODGE
            || newState == CoralState.PREPARE_BACK_ALGAE_DISLODGE;
      case FRONT_ALGAE_DISLODGE:
      case BACK_ALGAE_DISLODGE:
      case L1_SCORE:
      case L2_SCORE:
      case L3_SCORE:
      case L4_SCORE:
        return newState == CoralState.PREPARE_STOW_LOW;
      default:
        return false;
    }
  }

  /** Applies the preset values for a given state (elevator height, arm angle, etc.). */
  private void applyPreset(CoralState state) {
    CoralSystemPresets preset = getPresetForState(state);
    if (preset == null) {
      Logger.recordOutput("Coral/FSM ERROR", "No preset found for state " + state);
      return;
    }

    // Log requested setpoints
    Logger.recordOutput("Coral/Elevator Requested Height", preset.getElevatorHeight());
    Logger.recordOutput("Coral/Arm Requested Angle", preset.getArmAngle());

    // Move the elevator
    coralSubsystem.getElevator().setPosition(preset.getElevatorHeight());

    // Move the arm, unless we need to keep the angle
    if (!Double.isNaN(preset.getArmAngle())) {
      coralSubsystem.getArm().setAngleDegrees(preset.getArmAngle());
    }

    // Handle end effector actions
    if (state == CoralState.PREPARE_CORAL_STATION_PICKUP) {
      coralSubsystem.getEndEffector().runOpenLoop(1.0);
    } else if (state.name().contains("PREPARE_L")) { // Any scoring state
      coralSubsystem.getEndEffector().runOpenLoop(-1.0);
    } else {
      coralSubsystem.getEndEffector().stop();
    }
  }

  /** Finds the correct CoralSystemPreset for a given CoralState */
  private CoralSystemPresets getPresetForState(CoralState state) {
    for (CoralSystemPresets preset : CoralSystemPresets.values()) {
      if (preset.getState() == state) {
        return preset;
      }
    }
    return null; // No preset found, return null
  }

  /** Returns whether the arm and elevator have reached their goals. */
  private boolean isMovementComplete() {
    boolean complete =
        coralSubsystem.getElevator().isAtGoal() && coralSubsystem.getArm().isAtGoal();
    Logger.recordOutput("Coral/Motion Complete", complete);
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

  /** Determines if a state is a final (fully reached) state. */
  private boolean isFinalState(CoralState state) {
    return !isPrepareState(state);
  }

  /** Returns the current state. */
  public CoralState getCurrentState() {
    return currentState;
  }
}
