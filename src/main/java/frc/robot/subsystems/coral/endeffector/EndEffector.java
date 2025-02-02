package frc.robot.subsystems.coral.endeffector;

public class EndEffector {
  private final EndEffectorIO io;
  private boolean isActive;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  /** Update method to refresh sensor inputs and update internal state. */
  public void update() {
    // Create an inputs container and update it via the I/O implementation.
    EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
    io.updateInputs(inputs);
    // Use the inputs to update internal state.
    isActive = inputs.isActive;
    // Optionally, you can log or process additional values from inputs.
  }

  /**
   * Sets the active state of the end effector.
   *
   * @param active true to activate, false to deactivate.
   */
  public void setActive(boolean active) {
    io.setActive(active);
  }

  /**
   * Returns whether the end effector is currently active.
   *
   * @return true if active; false otherwise.
   */
  public boolean isActive() {
    return isActive;
  }

  /**
   * Commands the end effector to run in open-loop mode.
   *
   * @param output percent output (-1.0 to 1.0)
   */
  public void runOpenLoop(double output) {
    io.runOpenLoop(output);
  }

  public void stop() {
    io.stop();
  }
}
