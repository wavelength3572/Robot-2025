package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double setpoint = 0.0;
    public double elevatorHeight = 0.0;
    public double elevatorHeightCalc = 0.0;
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double pidOutput = 0.0;
    public double feedforwardOutput = 0.0;

    public double followerPositionRotations = 0.0;
    public double followerVelocityRPM = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setPosition(double requestedPosition, double arbFF) {}

  public default double getHeightInMeters() {
    return 0.0;
  }

  public default double getSetpointInMeters() {
    return 0.0;
  }

  public default void setPIDValues(
      double kP, double kD, double kF, double VelocityMax, double AccelerationMax) {}
}
