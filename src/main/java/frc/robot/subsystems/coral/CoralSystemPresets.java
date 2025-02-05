package frc.robot.subsystems.coral;

public enum CoralSystemPresets {
  STARTUP(0, 90, CoralState.STARTUP),
  STOW(22, 90, CoralState.STOW),
  PICKUP(32.5196, 230, CoralState.PICKUP),
  SCORE_LEVEL_1(0.00000, 90, CoralState.L1_SCORE),
  SCORE_LEVEL_2(18.8976, 45, CoralState.L2_SCORE),
  SCORE_LEVEL_3(36.6141, 45, CoralState.L3_SCORE),
  SCORE_LEVEL_4(59.0551, 45, CoralState.L4_SCORE),
  DISLODGE_LEVEL_1(23.6220, 45, CoralState.L1_DISLODGE),
  DISLODGE_LEVEL_2(47.2440, 45, CoralState.L2_DISLODGE);
  // SCORE_LEVEL_2_Straight(Units.inchesToMeters(12), 90, CoralState.PREPARE_L2_SCORE),
  // SCORE_LEVEL_3_Straight(Units.inchesToMeters(), 90, CoralState.PREPARE_L2_SCORE),
  // highest height 65 inches

  private final double elevatorHeight;
  private final double armAngle;
  private final CoralState state;

  CoralSystemPresets(double elevatorHeightINCHES, double armAngleDEGREES, CoralState state) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
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
    STARTUP,
    STOW,
    PICKUP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    L1_DISLODGE,
    L2_DISLODGE
  }
}
