package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 90, CoralState.STARTUP),
  STOW(22, 105, CoralState.STOW),
  PICKUP(36.0, 230, CoralState.PICKUP),
  SCORE_LEVEL_1(0.00000, 90, CoralState.L1_SCORE),
  SCORE_LEVEL_2(21.0, 70, CoralState.L2_SCORE),
  SCORE_LEVEL_3(37, 70, CoralState.L3_SCORE),
  SCORE_LEVEL_4(65, 55, CoralState.L4_SCORE),
  PREPARE_DISLODGE_LEVEL_1(15, 45, CoralState.PREP_L1_DISLODGE),
  FINAL_DISLODGE_LEVEL_1(22, 110, CoralState.FINAL_L1_DISLODGE),
  PREPARE_DISLODGE_LEVEL_2(25, 45, CoralState.PREP_L2_DISLODGE),
  FINAL_DISLODGE_LEVEL_2(32, 110, CoralState.FINAL_L2_DISLODGE);

  @Getter private final double elevatorHeight;
  @Getter private final double armAngle;
  @Getter private final CoralState state;

  CoralSystemPresets(double elevatorHeightINCHES, double armAngleDEGREES, CoralState state) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
    this.state = state;
  }

  public enum CoralState {
    STARTUP,
    STOW,
    PICKUP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    PREP_L1_DISLODGE,
    FINAL_L1_DISLODGE,
    PREP_L2_DISLODGE,
    FINAL_L2_DISLODGE,
  }
}
