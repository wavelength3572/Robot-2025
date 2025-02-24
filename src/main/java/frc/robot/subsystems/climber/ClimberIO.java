package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Relay;
import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double targetPosition = 0.0;
    public double currentPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public Relay.Value relayState;
    public CLIMB_STATE currentClimbState = CLIMB_STATE.STOWED;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void deployClimber() {}

  public default void climb() {}

  public default void stopClimber() {}

  public default void setRelayState(Relay.Value newState) {}

  public default boolean isClimberDeployed() {
    return false;
  }
}
