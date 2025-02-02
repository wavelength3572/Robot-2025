package frc.robot.subsystems.coral.endeffector;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the EndEffector I/O. This pattern follows the ElevatorIO style. */
public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean motorConnected = true;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean isActive = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {}

  /**
   * Run the end effector in open-loop mode. A positive output might intake a game piece, while a
   * negative output ejects it.
   *
   * @param output The percent output (from -1.0 to 1.0).
   */
  public default void runOpenLoop(double output) {}

  /**
   * Command the end effector using a voltage.
   *
   * @param volts The voltage to apply.
   */
  public default void runVolts(double volts) {}

  /** Stop the end effector. */
  public default void stop() {}

  /**
   * Run the end effector in closed-loop speed mode. This default implementation can be overridden
   * as needed.
   *
   * @param speed The target speed (interpreted as needed by your control scheme).
   */
  public default void runSpeed(double speed) {}

  /**
   * Set the active state of the end effector.
   *
   * @param active True if the end effector should be active (e.g. intaking), false otherwise.
   */
  public default void setActive(boolean active) {}
}
