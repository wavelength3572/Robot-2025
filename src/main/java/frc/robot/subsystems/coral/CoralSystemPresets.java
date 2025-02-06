package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 90, CoralState.STARTUP, 0.0, 0.0),
  STOW(22, 90, CoralState.STOW, 0.002, 0.0001),
  PICKUP(32.5196, 230, CoralState.PICKUP, 0.0, 0.0),
  SCORE_LEVEL_1(0.00000, 90, CoralState.L1_SCORE, 0.002, 0.0001),
  SCORE_LEVEL_2(21.0, 70, CoralState.L2_SCORE, 0.002, 0.002),
  SCORE_LEVEL_3(37, 70, CoralState.L3_SCORE, 0.001, 0.0001),
  SCORE_LEVEL_4(67, 55, CoralState.L4_SCORE, 0.0005, 0.0001),
  DISLODGE_LEVEL_1(23.6220, 45, CoralState.L1_DISLODGE, 0.0, 0.0),
  DISLODGE_LEVEL_2(47.2440, 45, CoralState.L2_DISLODGE, 0.0, 0.0);
  // SCORE_LEVEL_2_Straight(Units.inchesToMeters(12), 90, CoralState.PREPARE_L2_SCORE),
  // SCORE_LEVEL_3_Straight(Units.inchesToMeters(), 90, CoralState.PREPARE_L2_SCORE),
  // highest height 65 inches

  private final double elevatorHeight;
  private final double armAngle;
  private final CoralState state;
  @Getter private final double elevatorFF;
  @Getter private final double armFF;

  CoralSystemPresets(
      double elevatorHeightINCHES,
      double armAngleDEGREES,
      CoralState state,
      double elevatorFF,
      double armFF) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
    this.state = state;
    this.elevatorFF = elevatorFF;
    this.armFF = armFF;
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
