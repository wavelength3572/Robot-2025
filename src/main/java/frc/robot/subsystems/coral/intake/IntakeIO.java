package frc.robot.subsystems.coral.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double requestedSpeed = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double motorPosition = 0.0;
    public boolean limitSwitch = false;
    public boolean haveCoral = false;
    public double currentAmps = 0.0;
    public double Arm_TBE = 0.0;
    public double Arm_TBE_DEG = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void pullCoral() {}

  public default void pushCoral() {}

  public default void stopIntake() {}

  public default boolean haveCoral() {
    return false;
  }

  public default double get_Arm_TBE_DEG() {
    return 0.0;
  }

  public default void autoSetHaveCoral(boolean haveCoral) {}
}
