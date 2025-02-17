package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double requestedPosition = 0.0;
    public double appliedVolts = 0.0;
    public boolean climberDeployed = false;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void deployClimber() {}

  public default void stowClimber() {}

  public default void climb() {}

  public default boolean isClimberDeployed() {
    return false;
  }
}
