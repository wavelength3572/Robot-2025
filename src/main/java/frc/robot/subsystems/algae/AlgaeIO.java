package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {

    // Capture Motor Values
    public double captureRequestedSpeed = 0.0;
    public double captureVelocityRPM = 0.0;
    public double captureAppliedVolts = 0.0;
    public double captureCurrentAmps = 0.0;

    // Deploy Motor Values (Tracked in Both Degrees and Rotations)
    public double encoderRotations = 0.0; // Current deploy encoder rotations
    public double targetEncoderRotations = 0.0; // Target encoder rotations

    // Feedforward and Control Values
    public double armArbFF = 0.0; // Raw feedforward value

    // Motion Control Data
    public double velocityRPM = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;

    // Algae Detection
    public boolean algaeInRobot = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(AlgaeIOInputs inputs) {}

  /** Set intake motor speed (open-loop). */
  public default void setSpeed(double requestedSpeed) {}

  /** Set PID values dynamically for position control tuning. */
  public default void setPIDValues(
      double kP, double kD, double velocityMax, double accelerationMax) {}

  /** Run intake motor to pull in algae (game piece). */
  public default void pullAlgae() {}

  /** Run intake motor to push out algae. */
  public default void pushAlgae() {}

  /** Stop intake motor. */
  public default void stop() {}

  /** Deploy the arm to the predefined position. */
  public default void deployAlgae() {}

  /** Stow the arm to the predefined position. */
  public default void stowAlgae() {}

  public default double getDeployPosition() {
    return 0.0;
  }

  public default void setDeployPosition(double rotations) {}

  /** Check if an algae (game piece) is in the system. */
  public default boolean isAlgaeInRobot() {
    return false;
  }

  /** Manually set algae detection status. */
  public default void setAlgaeInRobot(boolean algaeInRobot) {}

  public default double getCurrentSpeedRPM() {
    return 0.0;
  }
}
