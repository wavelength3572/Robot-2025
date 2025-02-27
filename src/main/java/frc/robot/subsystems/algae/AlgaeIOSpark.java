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

  private SparkMax algaeCaptureMotor = new SparkMax(AlgaeConstants.algaeCaptureCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaeCaptureController = algaeCaptureMotor.getClosedLoopController();
  private RelativeEncoder algaeCaptureEncoder = algaeCaptureMotor.getEncoder();

  private SparkMax algaeDeployMotor = new SparkMax(AlgaeConstants.algaeDeployCanId, MotorType.kBrushless);
  private RelativeEncoder algaeDeployEncoder = algaeDeployMotor.getEncoder();

  private double captureEncoderValue = 0.0;

  private algaeIntakeState currentAlgIntakeState = algaeIntakeState.OFF;

  private double previousCaptureVel = 0.0;
  private double targetRPM = 0.0;
  private double detectionCount = 0;

  private boolean haveAlgae = false;

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
    inputs.captureVelocityRPM = algaeCaptureMotor.getEncoder().getVelocity();
    inputs.captureAppliedVolts = algaeCaptureMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.captureCurrentAmps = algaeCaptureMotor.getOutputCurrent();
    inputs.captureEncRotations = algaeCaptureEncoder.getPosition();

    // Logging values
    inputs.deployEncRotations = algaeDeployEncoder.getPosition();
    inputs.currentAngle = rotationsToAngle(inputs.deployEncRotations);

    inputs.armArbFF = Math.cos(Math.toRadians(inputs.currentAngle + 21.5)) * AlgaeConstants.deployPullBackFF;
    inputs.deployAppliedVolts = algaeDeployMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.deployCurrentAmps = algaeDeployMotor.getOutputCurrent();
    inputs.deployVelocityRPM = algaeDeployEncoder.getVelocity();

    inputs.currentIntakeState = currentAlgIntakeState;

    switch (currentAlgIntakeState) {
      case OFF:
        algaeCaptureMotor.setVoltage(0.0);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldVolts);
        previousCaptureVel = inputs.captureVelocityRPM;
        detectionCount = 0;
        break;
      case BURST:
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);

        detectionCount++;
        if (detectionCount > 50) {
          currentAlgIntakeState = algaeIntakeState.PULL;
          detectionCount = 0;
        }
        break;
      case PULL:
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldOutVolts);
        if (inputs.captureVelocityRPM - previousCaptureVel <= 10.0) {
          detectionCount++;
          if (detectionCount >= 10) {
            currentAlgIntakeState = algaeIntakeState.DETECT;
            // TargetRPM is the near max RPM value we think the intake roller reached
            // we'll use it to compare to the real time RPM to try and
            // determine if we captured an algae by looking for an RPM drop
            targetRPM = inputs.captureVelocityRPM;
            inputs.targetRPM = targetRPM;
          }
        } else {
          detectionCount = 0;
        }
        previousCaptureVel = inputs.captureVelocityRPM;
        break;
      case DETECT:
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldOutVolts);
        detectionCount = 0;
        if (targetRPM - inputs.captureVelocityRPM > 100) {
          captureEncoderValue = inputs.captureEncRotations;
          currentAlgIntakeState = algaeIntakeState.CAPTURE;
        }
        break;
      case CAPTURE:
        // This hold the capture motor position so
        // hopefully the algae doesn't move
        haveAlgae = true;
        algaeCaptureController.setReference(captureEncoderValue, ControlType.kPosition);
        detectionCount = 0;
        currentAlgIntakeState = algaeIntakeState.PULL_ARM;
        break;
      case PULL_ARM:
        algaeDeployMotor.setVoltage(inputs.armArbFF);
        break;
      case PUSH:
        algaeDeployMotor.setVoltage(AlgaeConstants.deployPushAlgaeVolts);
        algaeCaptureMotor.setVoltage(AlgaeConstants.capturePushVolts);
        break;
      case STOW:
        haveAlgae = false;
        algaeCaptureMotor.setVoltage(0.0);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployPullBackVolts);
        detectionCount++;
        if (detectionCount > 25) { // about .5 seconds
          currentAlgIntakeState = algaeIntakeState.OFF;
          algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldVolts);
          detectionCount = 0;
        }
        break;
      case MANUAL:
        break;
      case CLIMB:
        algaeCaptureMotor.setVoltage(AlgaeConstants.capturePushVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);
        detectionCount++;
        if (detectionCount > 50) {
          // Purposly setting speed here because I got some twitches in algae arm duing
          // testing.
          algaeDeployMotor.set(0.0);
          algaeCaptureMotor.set(0.0);
        }
        break;
      default:
        break;
    }
    // Game piece detection
    inputs.haveAlgae = haveAlgae; // Placeholder; add sensor logic if needed
  }

  public double angleToRotations(double angle) {
    return (angle / 360.0) * AlgaeConstants.kAlgaeDeployGearing;
  }

  public double rotationsToAngle(double rotations) {
    return (rotations / AlgaeConstants.kAlgaeDeployGearing) * 360.0;
  }

  public void algaeInClimbPosition() {
    detectionCount = 0;
    currentAlgIntakeState = algaeIntakeState.CLIMB;
  }

  @Override
  public void setCapturePIDValues(double kP, double kD) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(kP, 0.0, kD, 0.0);
    algaeCaptureMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void pushAlgae() {
    detectionCount = 0;
    currentAlgIntakeState = algaeIntakeState.PUSH;
  }

  @Override
  public void pullAlgae() {
    algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
    algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);
    detectionCount = 0;
    previousCaptureVel = algaeCaptureMotor.getEncoder().getVelocity();
    currentAlgIntakeState = algaeIntakeState.BURST;
  }

  @Override
  public void stowAlgae() {
    detectionCount = 0;
    currentAlgIntakeState = algaeIntakeState.STOW;
  }

  @Override
  public void setDeployVolts(double requestedVolts) {
    currentAlgIntakeState = algaeIntakeState.MANUAL;
    algaeDeployMotor.setVoltage(requestedVolts);
  }

  @Override
  public void setIntakeVolts(double requestedVolts) {
    // algaeCaptureMotor.setVoltage(requestedVolts);
  }

  public double getDeployPositionAngle() {
    return rotationsToAngle(algaeDeployEncoder.getPosition());
  }

  public boolean haveAlgae() {
    return haveAlgae;
  }
}
