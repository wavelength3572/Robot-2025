package frc.robot.subsystems.coral.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double setpoint = 0.0;
    public double encoderPositionRotations = 0.0;
    public double armAngleDegrees = 0.0;
    public double armAngleDegreesCalc = 0.0;
    public double armAngleRad = 0.0;
    public double armAngleRadCalc = 0.0;
    public double motorVelocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double pidOutput = 0.0;
    public double feedforwardOutput = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngleDegrees(double requestedAngleDegrees) {}

  public default void holdArmAngle() {}

  public default void setPIDValues(
      double kP, double kI, double kD, double VelocityMax, double AccelerationMax) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void stop() {}

  public default double getAngleInRadians() {
    return 0.0;
  }

  public default double getAngleInDegrees() {
    return 0.0;
  }

  public default boolean isAtGoal() {
    return false;
  }
}
