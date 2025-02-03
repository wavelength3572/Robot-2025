package frc.robot.subsystems.coral;

public enum CoralSystemPresets {
  STOW(0, 90, CoralState.PREPARE_STOW),
  STOW_LOW(.52, 270, CoralState.PREPARE_STOW_LOW),
  SAFE_CARRIAGE_POSITION(.52, Double.NaN, CoralState.PREPARE_SAFE_CARRIAGE_POSITION),

  PICKUP(0.826, 230, CoralState.PICKUP),

  SCORE_LEVEL_1(.26, 45, CoralState.PREPARE_L1_SCORE),
  SCORE_LEVEL_2(.48, 47, CoralState.PREPARE_L2_SCORE),
  SCORE_LEVEL_3(.93, 47, CoralState.PREPARE_L3_SCORE),
  SCORE_LEVEL_4(1.5, 45, CoralState.PREPARE_L4_SCORE),

  // Algae dislodge positions
  FRONT_ALGAE_DISLODGE_LEVEL_1(0.6, 45, CoralState.PREPARE_FRONT_ALGAE_DISLODGE),
  FRONT_ALGAE_DISLODGE_LEVEL_2(1.2, 85, CoralState.PREPARE_FRONT_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_1(0.6, 195, CoralState.PREPARE_BACK_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_2(1.0, 195, CoralState.PREPARE_BACK_ALGAE_DISLODGE);

  private final double elevatorHeight;
  private final double armAngle;
  private final CoralState state;

  CoralSystemPresets(double elevatorHeight, double armAngle, CoralState state) {
    this.elevatorHeight = elevatorHeight;
    this.armAngle = armAngle;
    this.state = state;
  }

  public double getElevatorHeight() {
    return elevatorHeight;
  }

  public double getArmAngle() {
    return armAngle;
  }

  public CoralState getState() {
    return state;
  }

  public enum CoralState {
    // Prepare states (Moving towards final states)
    PREPARE_STOW,
    PREPARE_STOW_LOW,
    PREPARE_SAFE_CARRIAGE_POSITION,
    PREPARE_CORAL_STATION_PICKUP,
    PREPARE_L1_SCORE,
    PREPARE_L2_SCORE,
    PREPARE_L3_SCORE,
    PREPARE_L4_SCORE,
    PREPARE_FRONT_ALGAE_DISLODGE,
    PREPARE_BACK_ALGAE_DISLODGE,

    // Final states (Robot is fully in position)
    STOW,
    STOW_LOW,
    SAFE_CARRIAGE_POSITION,
    PICKUP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    FRONT_ALGAE_DISLODGE,
    BACK_ALGAE_DISLODGE
  }
}
