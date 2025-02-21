package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double requestedPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public CLIMB_STATE currentClimbState = CLIMB_STATE.STOWED;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void deployClimber() {}

  public default void stowClimber() {}

  public default void stopClimber() {}

  public default void climb() {}

  public default boolean isClimberDeployed() {
    return false;
  }
}
