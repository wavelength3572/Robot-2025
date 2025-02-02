package frc.robot.subsystems.coral.arm;

/**
 * The Arm class is a component of the CoralSubsystem. It uses an injected ArmIO to perform
 * low-level control and sensor updates.
 */
public class Arm {
  private final ArmIO io;
  private double currentAngle;

  /**
   * Constructs the Arm with the given I/O implementation.
   *
   * @param io the ArmIO implementation (hardware or simulation)
   */
  public Arm(ArmIO io) {
    this.io = io;
  }

  /**
   * Update method to be called periodically by the CoralSubsystem. This updates sensor inputs and
   * stores the current angle.
   */
  public void update() {
    io.updateInputs();
    currentAngle = io.getAngle();
  }

  /**
   * Commands the arm to move toward the given angle.
   *
   * @param angle the target angle (units as defined by your implementation)
   */
  public void setAngle(double angle) {
    io.setAngle(angle);
  }

  /**
   * Returns the current angle of the arm.
   *
   * @return the current angle
   */
  public double getAngle() {
    return currentAngle;
  }

  /**
   * The ArmIO interface defines the low-level I/O operations for the arm. Implementations of this
   * interface can be used for hardware or simulation.
   */
  public interface ArmIO {
    /** Update sensor inputs and internal state. */
    void updateInputs();

    /** Get the current arm angle (in radians or degrees as you define it). */
    double getAngle();

    /** Command the arm to move toward a specified angle. */
    void setAngle(double angle);
  }
}
