package frc.robot.subsystems.coral;

import lombok.Getter;

public enum CoralSystemPresets {
  STARTUP(0, 105),
  STOW(22, 105),
  ARMSAFE(22, 85),
  PICKUP(39.0, 233),
  PICKUPFAR(37.0, 233), // TOF > 300
  L1(0.0, 92),
  L2(16.0, 77),
  L3(33, 77),
  L4(65, 45),
  PREL4(65, 85),
  PREPARE_DISLODGE_PART1_LEVEL_1(15, 45),
  PREPARE_DISLODGE_PART2_LEVEL_1(20, 45),
  FINAL_DISLODGE_LEVEL_1(27, 110),
  PREPARE_DISLODGE_PART1_LEVEL_2(30, 45),
  PREPARE_DISLODGE_PART2_LEVEL_2(35, 45),
  FINAL_DISLODGE_LEVEL_2(37, 110),
  CLIMB(0, 160);

  @Getter private final double elevatorHeight;
  @Getter private final double armAngle;

  CoralSystemPresets(double elevatorHeightINCHES, double armAngleDEGREES) {
    this.elevatorHeight = elevatorHeightINCHES;
    this.armAngle = armAngleDEGREES;
  }
}
