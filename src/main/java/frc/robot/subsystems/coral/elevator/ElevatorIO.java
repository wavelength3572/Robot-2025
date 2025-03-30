package frc.robot.subsystems.coral.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double setpoint = 0.0;
    public double elevatorHeight = 0.0;
    public double elevatorHeightCalc = 0.0;
    public double leaderPositionRotations = 0.0;
    public double leaderVelocityRPM = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double feedforwardOutput = 0.0;

    public double followerPositionRotations = 0.0;
    public double followerVelocityRPM = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerError = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setPosition(double requestedPosition) {}

  public default void recoverElevator() {}

  public default void clearElevatorError() {}

  public default double getHeightInMeters() {
    return 0.0;
  }

  public default double getSetpointInMeters() {
    return 0.0;
  }

  public default void runCharacterization(double output) {}

  public default double getFFCharacterizationVelocity() {
    return 0.0;
  }

  public default void setPIDValues(
      double kP, double kD, double VelocityMax, double AccelerationMax) {}
}
