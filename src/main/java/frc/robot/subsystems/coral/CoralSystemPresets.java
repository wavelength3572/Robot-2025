package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 105),
  STOW(22, 105),
  ARMSAFE(22, 85),
  PICKUP(39.0, 233),
  PRE_PICKUP(39.0, 115),
  PICKUPFAR(37.0, 233),
  L1_STOW(0.0, 92),
  L1_SCORE(22.0, 240),
  L2(16.0, 77),
  L3(33, 77),
  L4(65, 45),
  PRE_SCORE(39.0, 85),
  PREL4(39.0, 105),
  AUTO_START_L4(65, 65),
  AUTO_SCORE_END(45, 85),
  PREPARE_DISLODGE_PART1_LEVEL_1(15, 45),
  PREPARE_DISLODGE_PART2_LEVEL_1(20, 45),
  FINAL_DISLODGE_LEVEL_1(27, 110),
  PREPARE_DISLODGE_PART1_LEVEL_2(30, 45),
  PREPARE_DISLODGE_PART2_LEVEL_2(38, 45),
  FINAL_DISLODGE_LEVEL_2(42, 110),
  CLIMB(0, 160);

  @Getter private final double elevatorHeight;
  @Getter private final double armAngle;

  CoralSystemPresets(double elevatorHeightINCHES, double armAngleDEGREES) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
  }
}
