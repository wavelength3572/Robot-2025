package frc.robot.subsystems.coral.arm;

import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm {
  // The low-level I/O for the arm (hardware or simulation)
  private final ArmIO io;
  // An inputs object for automatic logging
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // Tunable PID constants for the arm; these can be adjusted at runtime.
  private static final LoggedTunableNumber ArmKp =
      new LoggedTunableNumber("Arm/kP", ArmConstants.kArmKp);
  private static final LoggedTunableNumber ArmKi =
      new LoggedTunableNumber("Arm/kI", ArmConstants.kArmKi);
  private static final LoggedTunableNumber ArmKd =
      new LoggedTunableNumber("Arm/kD", ArmConstants.kArmKd);

  /**
   * Constructs the Arm using the provided ArmIO implementation.
   *
   * @param io the ArmIO implementation (hardware or simulation)
   */
  public Arm(ArmIO io) {
    this.io = io;
  }

  /**
   * Update method that should be called periodically (by the CoralSubsystem, for example). This
   * updates sensor inputs and logs them.
   */
  public void update() {
    // If any PID tunable values have changed, update the low-level IO.
    // Note: We now pass in the velocity and acceleration constants from
    // ArmConstants.
    if (ArmKp.hasChanged(hashCode())
        || ArmKi.hasChanged(hashCode())
        || ArmKd.hasChanged(hashCode())) {
      io.setPIDValues(
          ArmKp.get(), ArmKi.get(), ArmKd.get(), ArmConstants.kArmVel, ArmConstants.kArmAcc);
    }
    // Update sensor inputs
    io.updateInputs(inputs);
    // Log the inputs for diagnostics.
    Logger.processInputs("Arm", inputs);
  }

  /**
   * Commands the arm to move toward the specified target angle (in degrees).
   *
   * @param requestedAngleDegrees the target angle in degrees
   */
  public void setAngleDegrees(double requestedAngleDegrees) {
    io.setAngleDegrees(requestedAngleDegrees);
  }

  /**
   * Returns the current angle of the arm in radians.
   *
   * @return the current arm angle in radians
   */
  public double getAngleInRadians() {
    return io.getAngleInRadians();
  }

  public void holdArmAngle() {
    io.holdArmAngle();
  }

  /**
   * Returns the current angle of the arm in degrees.
   *
   * @return the current arm angle in degrees
   */
  public double getAngleInDegrees() {
    return io.getAngleInDegrees();
  }

  public double getCalibratedAngleDegrees() {
    return getAngleInDegrees() - ArmConstants.VISUALIZATION_CALIBRATION_OFFSET_DEGREES;
  }

  public void calibrateArmOffset() {
    double currentAngle = getAngleInDegrees();
    ArmConstants.VISUALIZATION_CALIBRATION_OFFSET_DEGREES = currentAngle;
    Logger.recordOutput(
        "Arm Angle Calibrated", ArmConstants.VISUALIZATION_CALIBRATION_OFFSET_DEGREES);
  }

  public boolean isAtGoal() {
    return io.isAtGoal();
  }
}
