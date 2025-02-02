package frc.robot.subsystems.coral.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ArmIOSpark implements ArmIO {
  private final SparkFlex armMotor = new SparkFlex(ArmConstants.motorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController armClosedLoopController =
      armMotor.getClosedLoopController();
  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  // Instead of storing the target in radians, we store it in native units (motor rotations)
  private double armCurrentTargetNative = 0.0;

  public ArmIOSpark() {
    armMotor.configure(
        ArmConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Set the closed-loop reference using the native units (motor rotations)
    armClosedLoopController.setReference(
        armCurrentTargetNative, ControlType.kMAXMotionPositionControl);

    // Convert native units to physical arm angle for logging.
    // motor rotations -> (divide by gear ratio) gives arm rotations,
    // then multiply by 2π to obtain radians.
    double armAngleRadiansCalc =
        (armEncoder.getPosition() / ArmConstants.kArmGearing) * (2.0 * Math.PI);

    // For logging, you might want to report the physical angle in radians.
    inputs.setpoint = (armCurrentTargetNative / ArmConstants.kArmGearing) * (2.0 * Math.PI);
    inputs.encoderPositionRotations = armEncoder.getPosition();
    inputs.armAngleRadCalc = armAngleRadiansCalc;
    inputs.motorVelocityRPM = armEncoder.getVelocity() / ArmConstants.kArmGearing;
    inputs.appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = armMotor.getOutputCurrent();
  }

  // Convert desired angle (in degrees) into native units (motor rotations)
  @Override
  public void setAngleDegrees(double requestedAngleDegrees) {
    double requestedRadians = Units.degreesToRadians(requestedAngleDegrees);
    // Calculate the corresponding motor rotations.
    // Formula: motor rotations = (desired arm angle in radians / 2π) * gear ratio
    this.armCurrentTargetNative = (requestedRadians / (2.0 * Math.PI)) * ArmConstants.kArmGearing;
  }

  @Override
  public double getAngleInRadians() {
    // Convert the current encoder position (motor rotations) into the physical arm angle in
    // radians.
    double armRotations = armEncoder.getPosition() / ArmConstants.kArmGearing;
    return armRotations * (2.0 * Math.PI);
  }

  @Override
  public double getAngleInDegrees() {
    return Units.radiansToDegrees(getAngleInRadians());
  }

  @Override
  public void setPIDValues(
      double kP, double kD, double kF, double VelocityMax, double AccelerationMax) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pidf(kP, 0.0, kD, kF)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(VelocityMax)
        .maxAcceleration(AccelerationMax)
        .allowedClosedLoopError(0.1);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void holdArmAngle() {
    // Read the current encoder position (in motor rotations)
    double currentNative = armEncoder.getPosition();

    // Update the internal target to match the current position
    armCurrentTargetNative = currentNative;

    // Ensure the closed-loop controller starts tracking this position
    armClosedLoopController.setReference(
        armCurrentTargetNative, ControlType.kMAXMotionPositionControl);
  }
}
