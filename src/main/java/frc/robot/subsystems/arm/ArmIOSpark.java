package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ArmIOSpark implements ArmIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex armMotor = new SparkFlex(ArmConstants.canId, MotorType.kBrushless);
  private SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations =
      ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  public ArmIOSpark() {
    armMotor.configure(
        ArmConfigs.ArmSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    armEncoder.setPosition(ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armClosedLoopController.setReference(
        armTargetEncoderRotations,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ArmConstants.kArmkG * Math.cos(Units.degreesToRadians(armTargetDEG)),
        ArbFFUnits.kVoltage);
    inputs.targetAngleDEG = armTargetDEG;
    inputs.currentAngleDEG = armEncoder.getPosition() * 360.0 / ArmConstants.kArmGearing;
    inputs.targetEncoderRotations = this.armTargetEncoderRotations;
    inputs.encoderRotations = armEncoder.getPosition();
    inputs.velocityRPM = armEncoder.getVelocity();
    inputs.appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = armMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  @Override
  public void setAngleDEG(double requestedPosition, double requestedArbFF) {
    this.armTargetDEG = requestedPosition;
    this.armTargetEncoderRotations = this.armTargetDEG * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public double getAngleDEG() {
    return this.armTargetEncoderRotations * (1.0 / ArmConstants.kArmGearing) * 360.0;
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(kP, 0.0, kD, 0.0);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

