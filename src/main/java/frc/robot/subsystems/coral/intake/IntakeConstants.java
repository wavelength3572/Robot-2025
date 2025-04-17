package frc.robot.subsystems.coral.intake;

public class IntakeConstants {
  public static final int canId = 20;
  public static final int intakeCurrentLimit = 30;

  public static final double intakeInSpeed = 0.50; // Original -0.6
  public static final double intakeOutSpeed = -0.75; // Original 1.0

  public static enum INTAKE_STATE {
    OFF,
    PUSH,
    PULL,
    FORCE_PULL
  }
}
