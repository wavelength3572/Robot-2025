package frc.robot.subsystems.algae;

import frc.robot.subsystems.algae.AlgaeConstants.algaeIntakeState;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {

    // Capture Motor Values
    public double captureVelocityRPM = 0.0;
    public double captureAppliedVolts = 0.0;
    public double captureCurrentAmps = 0.0;
    public double captureEncRotations = 0.0; // Current capture encoder rotations
    public double captureTargetRotations = 0.0;

    // Deploy Motor Values (Tracked in Both Degrees and Rotations)
    public double deployEncRotations = 0.0; // Current deploy encoder rotations
    public double currentAngle = 0.0;

    public algaeIntakeState currentIntakeState = algaeIntakeState.OFF;

    // Feedforward and Control Values
    public double armArbFF = 0.0; // Raw feedforward value

    // Motion Control Data
    public double deployVelocityRPM = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;

    // Algae Detection
    public boolean haveAlgae = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(AlgaeIOInputs inputs) {}

  /** Set deploy motor speed (open-loop). */
  public default void setDeployVolts(double requestedVolts) {}

  public default void setIntakeVolts(double requestedVolts) {}

  /** Set PID values dynamically for position control tuning. */
  public default void setCapturePIDValues(double kP, double kD) {}

  /** Run intake motor to pull in algae (game piece). */
  public default void pullAlgae() {}

  /** Run intake motor to push out algae. */
  public default void pushAlgae() {}

  /** Stop running stuff and pull in the arm. */
  public default void stowAlgae() {}

  public default void algaeInClimbPosition() {}

  public default double getDeployPositionAngle() {
    return 0.0;
  }

  /** Check if an algae (game piece) is in the system. */
  public default boolean haveAlgae() {
    return false;
  }

  /** Manually set algae detection status. */
  public default void setHaveAlgae(boolean haveAlgae) {}

  public default double getCurrentSpeedRPM() {
    return 0.0;
  }
}
