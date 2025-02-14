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

  private double targetAngleDEG = AlgaeConstants.algaeStartAngle;
  private double targetEncoderRotations =
      AlgaeConstants.algaeStartAngle * AlgaeConstants.kAlgaeDeployGearing / 360.0;

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
    algaeDeployEncoder.setPosition(
        AlgaeConstants.algaeStartAngle * AlgaeConstants.kAlgaeDeployGearing / 360.0);
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Capture Motor Inputs
    inputs.captureRequestedSpeed = algaeCaptureMotor.get();
    inputs.captureVelocityRPM = algaeCaptureMotor.getEncoder().getVelocity();
    inputs.captureAppliedVolts =
        algaeCaptureMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.captureCurrentAmps = algaeCaptureMotor.getOutputCurrent();

    // Convert encoder rotations to degrees
    inputs.currentAngleDEG =
        algaeDeployEncoder.getPosition() * 360.0 / AlgaeConstants.kAlgaeDeployGearing;

    // Compute feedforward torque compensation
    double armFeedforward =
        AlgaeConstants.kAlgaeDeployKf * Math.cos(Math.toRadians(inputs.currentAngleDEG));

    // Closed-loop position control for deploy motor
    algaeDeployController.setReference(
        targetEncoderRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        0.0 // for now don't put feedforward in
        );

    // Logging values
    inputs.targetAngleDEG = targetAngleDEG;
    inputs.targetEncoderRotations = targetEncoderRotations;
    inputs.encoderRotations = algaeDeployEncoder.getPosition();
    inputs.armArbFF = AlgaeConstants.kAlgaeDeployKf;
    inputs.armArbFF_COS = armFeedforward;

    // Game piece detection
    inputs.algaeInRobot = false; // Placeholder; add sensor logic if needed
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition) {
    this.targetAngleDEG = requestedPosition;
    this.targetEncoderRotations = this.targetAngleDEG * AlgaeConstants.kAlgaeDeployGearing / 360.0;
  }

  @Override
  public double getTargetAngleDEG() {
    return this.targetAngleDEG;
  }

  @Override
  public double getCurrentAngleDEG() {
    return algaeDeployEncoder.getPosition() * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pidf(kP, 0.0, kD, 0.0)
        .maxMotion
        .maxVelocity(VelocityMax)
        .maxAcceleration(AccelerationMax)
        .allowedClosedLoopError(0.1);
    algaeDeployMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void deployAlgae() {
    setTargetAngleDEG(AlgaeConstants.algaeDeployPosition);
  }

  @Override
  public void stowAlgae() {
    setTargetAngleDEG(AlgaeConstants.algaeStowPosition);
  }

  @Override
  public double getCurrentSpeedRPM() {
    return algaeCaptureMotor.getEncoder().getVelocity();
  }
}
