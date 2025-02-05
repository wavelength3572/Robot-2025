package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double requestedSpeed = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double Arm_TBE = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified speed. */
  public default void setSpeed(double requestedSpeed) {}
}
