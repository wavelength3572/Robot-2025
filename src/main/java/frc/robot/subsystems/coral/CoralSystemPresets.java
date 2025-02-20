package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 105, CoralState.STARTUP),
  STOW(22, 105, CoralState.STOW),
  PICKUP(40.0, 233, CoralState.PICKUP),
  PICKUPFAR(37.0, 233, CoralState.PICKUPFAR), // TOF > 300
  L1(0.00000, 92, CoralState.L1_SCORE),
  L2(14.0, 77, CoralState.L2_SCORE),
  L3(31, 77, CoralState.L3_SCORE),
  L4(65, 45, CoralState.L4_SCORE),
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
    PICKUPFAR,
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
