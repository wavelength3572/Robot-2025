package frc.robot.subsystems.coral.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.Elastic;
import frc.robot.util.RobotStatus;

public class ArmIOMMSpark implements ArmIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the
  // elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex armMotor = new SparkFlex(ArmConstants.canId, MotorType.kBrushless);
  private SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations = angleToRotations(armTargetDEG);

  private boolean TBE_Valid = true;
  private boolean ARM_STUCK_ERROR = false;
  private double ARM_STUCK_ERROR_COUNT = 0;

  private double armArbFF = ArmConstants.kArmKfNoCoral;

  private boolean inArmRecoveryMode = false;

  public ArmIOMMSpark() {
    armMotor.configure(
        ArmConfigs.ArmSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    armEncoder.setPosition(armTargetEncoderRotations);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps = armMotor.getOutputCurrent();
    inputs.encoderRotations = armEncoder.getPosition();
    inputs.currentAngleDEG = rotationsToAngle(inputs.encoderRotations);

    if (inputs.currentAmps > 48.0) {
      ARM_STUCK_ERROR_COUNT++;
      if (ARM_STUCK_ERROR == false && ARM_STUCK_ERROR_COUNT >= 25) { // Approx .25 seconds
        ARM_STUCK_ERROR = true;
        // turn off motor immediatly;
        armMotor.set(0.0);
        // It will get set very soon to it's own current position
        // using MaxMotion
        Elastic.selectTab("ARM ERROR");
      }
    } else {
      ARM_STUCK_ERROR_COUNT = 0;
    }

    if (RobotStatus.haveCoral()) {
      armArbFF = ArmConstants.kArmKfCoral;
    } else {
      armArbFF = ArmConstants.kArmKfNoCoral;
    }

    if (ARM_STUCK_ERROR == true && inArmRecoveryMode == false) {
      // Arm is in error
      // Set the arm target to where it is right now.
      armTargetEncoderRotations = inputs.encoderRotations;
    }

    armClosedLoopController.setReference(
        armTargetEncoderRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        armArbFF * Math.cos(Math.toRadians(inputs.currentAngleDEG)));

    inputs.targetAngleDEG = rotationsToAngle(this.armTargetEncoderRotations);
    inputs.targetEncoderRotations = this.armTargetEncoderRotations;
    inputs.armArbFF = armArbFF;
    inputs.ARM_STUCK_ERROR = this.ARM_STUCK_ERROR;
    inputs.inArmRecoveryMode = this.inArmRecoveryMode;
    inputs.TBE_Valid = this.TBE_Valid;
    inputs.armArbFF_COS = armArbFF * Math.cos(Math.toRadians(inputs.currentAngleDEG));
    inputs.velocityRPM = armEncoder.getVelocity();
    inputs.appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = armMotor.getOutputCurrent();
  }

  @Override
  public void setInitialAngle(double initialDegree) {
    if (initialDegree >= 65 && initialDegree <= 144) {
      TBE_Valid = true;
      armTargetEncoderRotations = angleToRotations(initialDegree);
      armTargetDEG = initialDegree;
      armEncoder.setPosition(armTargetEncoderRotations);
    } else {
      // Bad ThroughBore Encoder Reading
      TBE_Valid = false;
    }
  }

  @Override
  public void setArbFFConstant(double volts) {
    if (volts >= -0.4 && volts <= 0.4) armArbFF = volts;
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition) {
    if (TBE_Valid && ARM_STUCK_ERROR == false) {
      this.armTargetDEG = requestedPosition;
      this.armTargetEncoderRotations = angleToRotations(requestedPosition);
    }
  }

  @Override
  public void recoverArm() {
    if (TBE_Valid) {
      inArmRecoveryMode = true;
      this.armTargetDEG = CoralSystemPresets.L1_SCORE.getArmAngle();
      this.armTargetEncoderRotations = angleToRotations(this.armTargetDEG);
    }
  }

  @Override
  public double getTargetAngleDEG() {
    return this.armTargetDEG;
  }

  @Override
  public double getCurrentArmDEG() {
    return rotationsToAngle(armEncoder.getPosition());
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    final SparkFlexConfig config = new SparkFlexConfig();
    config
        .closedLoop
        .pidf(kP, 0.0, kD, 0.0)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(VelocityMax)
        .maxAcceleration(AccelerationMax)
        .allowedClosedLoopError(0.1);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public boolean isArmInError() {
    return ARM_STUCK_ERROR;
  }

  @Override
  public void clearArmError() {
    ARM_STUCK_ERROR = false;
    ARM_STUCK_ERROR_COUNT = 0;
    inArmRecoveryMode = false;
  }

  public double angleToRotations(double angle) {
    return angle * ArmConstants.kArmGearing / 360.0;
  }

  public double rotationsToAngle(double rotations) {
    return (rotations / ArmConstants.kArmGearing) * 360.0;
  }
}
