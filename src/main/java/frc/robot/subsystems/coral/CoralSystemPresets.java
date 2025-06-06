package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 105),
  STOW(22, 105),
  ARMSAFE(22, 85),
  PICKUP(38, 233),
  PRE_PICKUP(38.0, 115),
  PICKUPFAR(35, 233),
  L1_STOW(0.0, 92),
  L1_SCORE(22.0, 240),
  L2(16.0, 77),
  STAGED_FOR_SCORING(16.0, 77),
  L3(33, 77),
  L4(64.5, 46),
  L2_FAR(16.0 + 3.5, 77), // moved up 2inches
  L3_FAR(33.0 + 3.5, 77), // moved up 2inches
  L4_FAR(64.5, 55), // not changed
  PRE_SCORE(38.0, 85),
  PREL4(39.0, 105), // Used going from Pickup to L4
  AUTO_START_L4(64.5, 46), // Used for first score in auto
  AUTO_SCORE_END(45, 85),
  PREPARE_DISLODGE_PART1_LEVEL_1(15, 45),
  PREPARE_DISLODGE_PART2_LEVEL_1(20, 45),
  FINAL_DISLODGE_LEVEL_1(23, 110),
  PREPARE_DISLODGE_PART1_LEVEL_2(30, 45),
  PREPARE_DISLODGE_PART2_LEVEL_2(38, 45),
  FINAL_DISLODGE_LEVEL_2(40, 110),
  CLIMB(0, 160);

  @Getter private final double elevatorHeight;
  @Getter private final double armAngle;

  CoralSystemPresets(double elevatorHeightINCHES, double armAngleDEGREES) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
  }
}
