package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotController;

public class AlgaeIOSpark implements AlgaeIO {

  private SparkMax algaeCaptureMotor =
      new SparkMax(AlgaeConstants.algaeCaptureCanId, MotorType.kBrushless);
  private SparkMax algaeDeployMotor =
      new SparkMax(AlgaeConstants.algaeDeployCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaeDeployController =
      algaeDeployMotor.getClosedLoopController();
  private RelativeEncoder algaeDeployEncoder = algaeDeployMotor.getEncoder();

  private double targetEncoderRotations = angleToRotations(AlgaeConstants.kAlgaeDeployInitalAngle);

  private double Aff = AlgaeConstants.kAlgaeDeployKf;

  public AlgaeIOSpark() {
    algaeCaptureMotor.configure(
        AlgaeConfigs.AlgaeSubsystem.algaeCaptureConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    algaeCaptureMotor.set(0.0);

    algaeDeployMotor.configure(
        AlgaeConfigs.AlgaeSubsystem.algaeDeployConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    algaeDeployEncoder.setPosition(angleToRotations(AlgaeConstants.kAlgaeDeployInitalAngle));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Capture Motor Inputs
    inputs.captureRequestedSpeed = algaeCaptureMotor.get();
    inputs.captureVelocityRPM = algaeCaptureMotor.getEncoder().getVelocity();
    inputs.captureAppliedVolts =
        algaeCaptureMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.captureCurrentAmps = algaeCaptureMotor.getOutputCurrent();

    // Logging values
    inputs.targetEncoderRotations = targetEncoderRotations;
    inputs.targetAngle = rotationsToAngle(targetEncoderRotations);
    inputs.encoderRotations = algaeDeployEncoder.getPosition();
    inputs.currentAngle = rotationsToAngle(inputs.encoderRotations);

    inputs.armArbFF = AlgaeConstants.kAlgaeDeployKf * Math.cos(Math.toRadians(inputs.currentAngle));
    inputs.deployAppliedVolts =
        algaeDeployMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.deployCurrentAmps = algaeDeployMotor.getOutputCurrent();

    // Closed-loop position control for deploy motor
    algaeDeployController.setReference(
        targetEncoderRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        AlgaeConstants.kAlgaeDeployKf * Math.cos(Math.toRadians(inputs.currentAngle)));

    // Game piece detection
    inputs.algaeInRobot = false; // Placeholder; add sensor logic if needed
  }

  public double angleToRotations(double angle) {
    return (angle / 360.0) * AlgaeConstants.kAlgaeDeployGearing;
  }

  public double rotationsToAngle(double rotations) {
    return (rotations / AlgaeConstants.kAlgaeDeployGearing) * 360.0;
  }

  @Override
  public void setPIDValues(
      double kP, double kD, double kFF, double VelocityMax, double AccelerationMax) {
    if (kFF >= -6.0 && kFF <= 6.0) {
      this.Aff = kFF;
    }
    final SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pidf(kP, 0.0, kD, 0.0)
        .maxMotion
        .maxVelocity(VelocityMax)
        .maxAcceleration(AccelerationMax)
        .allowedClosedLoopError(AlgaeConstants.kAlgaeDeployAllowableError);
    algaeDeployMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void deployAlgae() {
    targetEncoderRotations = angleToRotations(AlgaeConstants.algaeDeployPosition);
  }

  @Override
  public void stowAlgae() {
    targetEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);
  }

  @Override
  public void pushAlgae() {
    algaeCaptureMotor.set(AlgaeConstants.algaeOutSpeed);
  }

  @Override
  public void pullAlgae() {
    algaeCaptureMotor.set(AlgaeConstants.algaeInSpeed);
  }

  @Override
  public void stopAlgae() {
    algaeCaptureMotor.set(0.0);
  }

  @Override
  public double getCurrentSpeedRPM() {
    return algaeCaptureMotor.getEncoder().getVelocity();
  }

  @Override
  public void setDeployPositionAngle(double angle) {
    targetEncoderRotations = angleToRotations(angle);
  }
}
