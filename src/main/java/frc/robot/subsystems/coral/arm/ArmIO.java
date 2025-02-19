package frc.robot.subsystems.coral.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double targetAngleDEG = 0.0;
    public double currentAngleDEG = 0.0;
    public double targetEncoderRotations = 0.0;
    public double encoderRotations = 0.0;
    public boolean TBE_Valid = false;
    public double armArbFF = 0.0;
    public double armArbFF_COS = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setTargetAngleDEG(double requestedPositionDEG) {}

  public default double getTargetAngleDEG() {
    return 0.0;
  }

  public default double getCurrentArmDEG() {
    return 0.0;
  }

  public default void setInitialAngle(double initialDegree) {}

  public default void setPIDValues(
      double kP, double kD, double VelocityMax, double AccelerationMax) {}
}
