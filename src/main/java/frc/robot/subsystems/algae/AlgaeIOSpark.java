package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.algae.AlgaeConstants.algaeIntakeState;

public class AlgaeIOSpark implements AlgaeIO {

  private SparkMax algaeCaptureMotor =
      new SparkMax(AlgaeConstants.algaeCaptureCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaeCaptureController =
      algaeCaptureMotor.getClosedLoopController();
  private RelativeEncoder algaeCaptureEncoder = algaeCaptureMotor.getEncoder();

  private SparkMax algaeDeployMotor =
      new SparkMax(AlgaeConstants.algaeDeployCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaeDeployController =
      algaeDeployMotor.getClosedLoopController();
  private RelativeEncoder algaeDeployEncoder = algaeDeployMotor.getEncoder();

  private double targetEncoderRotations = angleToRotations(AlgaeConstants.kAlgaeDeployInitalAngle);

  private double captureEncoderValue = 0.0;

  private algaeIntakeState currentAlgIntakeState = algaeIntakeState.OFF;

  private double previousCaptureVel = 0.0;
  private double targetRPM = 0.0;
  private double detectionCount = 0;

  public AlgaeIOSpark() {
    algaeCaptureMotor.configure(
        AlgaeConfigs.AlgaeSubsystem.algaeCaptureConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    algaeCaptureMotor.set(0.0);
    algaeCaptureEncoder.setPosition(0.0);

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
    inputs.captureEncRotations = algaeCaptureEncoder.getPosition();

    // Logging values
    inputs.targetEncoderRotations = targetEncoderRotations;
    inputs.targetAngle = rotationsToAngle(targetEncoderRotations);
    inputs.targetEncoderRotations = algaeDeployEncoder.getPosition();
    inputs.currentAngle = rotationsToAngle(inputs.targetEncoderRotations);

    inputs.armArbFF = AlgaeConstants.kAlgaeDeployKf * Math.cos(Math.toRadians(inputs.currentAngle));
    inputs.deployAppliedVolts =
        algaeDeployMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.deployCurrentAmps = algaeDeployMotor.getOutputCurrent();
    inputs.deployVelocityRPM = algaeDeployEncoder.getVelocity();

    inputs.currentIntakeState = currentAlgIntakeState;
    inputs.targetEncoderRotations = captureEncoderValue;

    switch (currentAlgIntakeState) {
      case OFF:
        algaeCaptureMotor.setVoltage(0.0);
        algaeCaptureController.setReference(captureEncoderValue, ControlType.kPosition);
        previousCaptureVel = inputs.captureVelocityRPM;
        detectionCount = 0;
        break;
      case PULL:
        algaeCaptureMotor.setVoltage(8.0);
        algaeDeployMotor.setVoltage(0.4);
        detectionCount++;
        if (inputs.captureVelocityRPM != previousCaptureVel) {
          if (inputs.captureVelocityRPM - previousCaptureVel <= 10.0 && detectionCount > 10) {
            currentAlgIntakeState = algaeIntakeState.DETECT;
            targetRPM = inputs.captureVelocityRPM;
          }
        }
        previousCaptureVel = inputs.captureVelocityRPM;
        break;
      case DETECT:
        algaeCaptureMotor.setVoltage(8.0);
        detectionCount = 0;
        if (targetRPM - inputs.captureVelocityRPM > 75) {
          captureEncoderValue = inputs.captureEncRotations;
          currentAlgIntakeState = algaeIntakeState.CAPTURE;
        }
        break;
      case CAPTURE:
        algaeCaptureController.setReference(captureEncoderValue, ControlType.kPosition);
        // Pull arm back
        algaeDeployMotor.setVoltage(-4.0);
        detectionCount = 0;
        currentAlgIntakeState = algaeIntakeState.PULL_ARM;
        break;
      case PULL_ARM:
        if (detectionCount > 50) // about 1 seconds
        {
          currentAlgIntakeState = algaeIntakeState.HOLD_ARM;
        } else {
          algaeDeployMotor.setVoltage(-1);
        }
        detectionCount++;
        break;
      case HOLD_ARM:
        algaeDeployMotor.setVoltage(-1.5 + inputs.armArbFF);
        break;
      case PUSH:
        if (detectionCount < 100) // about 2 seconds
        {
          algaeDeployMotor.setVoltage(0.4);
          algaeCaptureMotor.setVoltage(-8.0);
        } else {
          algaeDeployMotor.setVoltage(0.0);
          algaeCaptureMotor.setVoltage(0.0);
          currentAlgIntakeState = algaeIntakeState.STOW;
        }
        detectionCount++;
        break;
      case STOW:
        algaeDeployMotor.setVoltage(0.0);
        algaeCaptureMotor.setVoltage(0.0);
        captureEncoderValue = inputs.captureEncRotations;
        currentAlgIntakeState = algaeIntakeState.OFF;
        break;
      default:
        break;
    }
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
  public void setDeployPIDValues(double kP, double kD) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(kP, 0.0, kD, 0.0);
    algaeDeployMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setCapturePIDValues(double kP, double kD) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(kP, 0.0, kD, 0.0);
    algaeCaptureMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void deployAlgae() {
    targetEncoderRotations = angleToRotations(AlgaeConstants.algaeDeployPosition);
  }

  @Override
  public void stowAlgae() {
    targetEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);
    currentAlgIntakeState = algaeIntakeState.STOW;
  }

  @Override
  public void pushAlgae() {
    detectionCount = 0;
    currentAlgIntakeState = algaeIntakeState.PUSH;
  }

  @Override
  public void pullAlgae() {
    algaeCaptureMotor.setVoltage(8.0);
    algaeDeployMotor.setVoltage(0.4);
    detectionCount = 0;
    previousCaptureVel = algaeCaptureMotor.getEncoder().getVelocity();
    currentAlgIntakeState = algaeIntakeState.PULL;
  }

  @Override
  public void stopAlgae() {
    currentAlgIntakeState = algaeIntakeState.OFF;
  }

  @Override
  public double getCurrentSpeedRPM() {
    return algaeCaptureMotor.getEncoder().getVelocity();
  }

  @Override
  public void setDeployVolts(double requestedVolts) {
    algaeDeployMotor.setVoltage(requestedVolts);
  }

  @Override
  public void setIntakeVolts(double requestedVolts) {
    // algaeCaptureMotor.setVoltage(requestedVolts);
  }

  @Override
  public void setDeployPositionAngle(double angle) {
    targetEncoderRotations = angleToRotations(angle);
    algaeDeployController.setReference(targetEncoderRotations, ControlType.kPosition);
  }
}
