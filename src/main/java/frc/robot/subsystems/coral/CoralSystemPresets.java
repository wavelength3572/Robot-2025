package frc.robot.subsystems.coral;

public enum CoralSystemPresets {
  STOW(0, 90, ActivityType.STOW),
  PICKUP(0.85, 220, ActivityType.PICKUP),

  SCORE_LEVEL_1(.26, 45, ActivityType.SCORING),
  SCORE_LEVEL_2(.48, 47, ActivityType.SCORING),
  SCORE_LEVEL_3(.93, 47, ActivityType.SCORING),
  SCORE_LEVEL_4(1.5, 45, ActivityType.SCORING),

  // Algae dislodge positions
  FRONT_ALGAE_DISLODGE_LEVEL_1(0.6, 45, ActivityType.FRONT_ALGAE_DISLODGE),
  FRONT_ALGAE_DISLODGE_LEVEL_2(1.2, 85, ActivityType.FRONT_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_1(0.6, 195, ActivityType.BACK_ALGAE_DISLODGE),
  BACK_ALGAE_DISLODGE_LEVEL_2(1.0, 195, ActivityType.BACK_ALGAE_DISLODGE);

  private final double elevatorHeight;
  private final double armAngle;
  private final ActivityType type;

  CoralSystemPresets(double elevatorHeight, double armAngle, ActivityType type) {
    this.elevatorHeight = elevatorHeight;
    this.armAngle = armAngle;
    this.type = type;
  }

  public double getElevatorHeight() {
    return elevatorHeight;
  }

  public double getArmAngle() {
    return armAngle;
  }

  public ActivityType getType() {
    return type;
  }

  public enum ActivityType {
    STOW,
    PICKUP,
    SCORING,
    FRONT_ALGAE_DISLODGE,
    BACK_ALGAE_DISLODGE
  }
}
