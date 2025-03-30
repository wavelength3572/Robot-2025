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
import frc.robot.subsystems.algae.AlgaeConstants.algaeIntakeState;
import frc.robot.util.LoggedTunableNumber;

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

  private double captureEncoderValue = 0.0;

  private algaeIntakeState currentAlgIntakeState = algaeIntakeState.OFF;

  private double previousArmAngle = 0.0;
  private double detectionCount = 0;

  private boolean haveAlgae = false;

  private static final LoggedTunableNumber deployAFF =
      new LoggedTunableNumber("Algae/deployAFF", AlgaeConstants.deployPullBackFF);

  private static final LoggedTunableNumber deploykP =
      new LoggedTunableNumber("Algae/deploykP", AlgaeConstants.kAlgaeDeployKp);

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

    if (deploykP.hasChanged(hashCode())) {
      final SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pidf(deploykP.get(), 0.0, 0.0, 0.0);
      algaeDeployMotor.configure(
          config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Capture Motor Inputs
    inputs.captureVelocityRPM = algaeCaptureMotor.getEncoder().getVelocity();
    inputs.captureAppliedVolts =
        algaeCaptureMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.captureCurrentAmps = algaeCaptureMotor.getOutputCurrent();
    inputs.captureEncRotations = algaeCaptureEncoder.getPosition();

    // Logging values
    inputs.deployEncRotations = algaeDeployEncoder.getPosition();
    inputs.currentAngle = rotationsToAngle(inputs.deployEncRotations);

    inputs.armArbFF = Math.cos(Math.toRadians(inputs.currentAngle + 21.5)) * deployAFF.get();
    inputs.deployAppliedVolts =
        algaeDeployMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.deployCurrentAmps = algaeDeployMotor.getOutputCurrent();
    inputs.deployVelocityRPM = algaeDeployEncoder.getVelocity();

    inputs.currentIntakeState = currentAlgIntakeState;

    switch (currentAlgIntakeState) {
      case OFF:
        haveAlgae = false;
        algaeDeployController.setReference(
            AlgaeConstants.algaeStowPosition,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            inputs.armArbFF);
        algaeCaptureMotor.setVoltage(0.0);
        // algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldVolts);
        previousArmAngle = inputs.currentAngle;
        detectionCount = 0;
        break;
      case BURST:
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);
        detectionCount = 0;
        previousArmAngle = inputs.currentAngle;
        currentAlgIntakeState = algaeIntakeState.PULL;
        break;
      case PULL:
        detectionCount++;
        if (detectionCount <= 50) {
          algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);
        } else {
          algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldOutVolts);
        }
        // Pull until we see the angle decrease, the arm is moving upward
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        if (previousArmAngle - inputs.currentAngle > 0.0) {
          // Checking to see if the arm is rising
          detectionCount++;
          if (detectionCount >= 3) {
            // We have detected 3 consecutive samples of the arm rising
            currentAlgIntakeState = algaeIntakeState.DETECT;
            detectionCount = 0;
            algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldOutVolts);
          }
        } else {
          detectionCount = 0;
        }
        previousArmAngle = inputs.currentAngle;
        break;
      case DETECT:
        algaeCaptureMotor.setVoltage(AlgaeConstants.captureIntakeVolts);
        algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldOutVolts);
        if (inputs.currentAngle - previousArmAngle > 0) {
          // Checking to see if the arm is lowering
          detectionCount++;
          if (detectionCount >= 3) {
            // We have detected 3 consecutive samples of the arm lowering
            captureEncoderValue = inputs.captureEncRotations;
            currentAlgIntakeState = algaeIntakeState.CAPTURE;
          }
        } else {
          detectionCount = 0;
        }
        previousArmAngle = inputs.currentAngle;
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
        currentAlgIntakeState = algaeIntakeState.OFF;
        // algaeDeployMotor.setVoltage(AlgaeConstants.deployPullBackVolts);

        // detectionCount++;
        // if (detectionCount > 25) { // about .5 seconds
        //   currentAlgIntakeState = algaeIntakeState.OFF;
        //   algaeDeployMotor.setVoltage(AlgaeConstants.deployHoldVolts);
        //   detectionCount = 0;
        // }
        break;
      case MANUAL:
        break;
      case CLIMB:
        haveAlgae = false;
        algaeDeployController.setReference(
            AlgaeConstants.algaeClimbPosition,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            inputs.armArbFF);
        algaeCaptureMotor.setVoltage(0.0);
        break;
        // case CLIMB:
        // detectionCount++;
        // if (detectionCount <= 50) { // About 1 second
        // algaeCaptureMotor.setVoltage(AlgaeConstants.capturePushVolts);
        // algaeDeployMotor.setVoltage(AlgaeConstants.deployBurstVolts);
        // } else {
        // // Purposly setting speed here because I got some twitches in algae arm duing
        // // testing.
        // algaeDeployMotor.set(0.0);
        // algaeCaptureMotor.set(0.0);
        // }
        // break;
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
