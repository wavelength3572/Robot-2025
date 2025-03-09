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
import frc.robot.util.RobotStatus;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

public class ArmIOMMSpark implements ArmIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the
  // elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex armMotor = new SparkFlex(ArmConstants.canId, MotorType.kBrushless);
  private SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations = ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  private boolean TBE_Valid = true;
  private boolean ARM_STUCK_ERROR = false;
  private double ARM_STUCK_ERROR_COUNT = 0;

  private double armArbFF = ArmConstants.kArmKfNoCoral;

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
    if (inputs.currentAmps > 48.0) {
      ARM_STUCK_ERROR_COUNT++;
      if (ARM_STUCK_ERROR_COUNT >= 13) { // Approx .25 seconds
        ARM_STUCK_ERROR = true;
        // turn off motor immediatly;
        armMotor.set(0.0);
      }
    } else {
      ARM_STUCK_ERROR_COUNT = 0;
    }
    inputs.currentAngleDEG = armEncoder.getPosition() * 360.0 / ArmConstants.kArmGearing;
    if (RobotStatus.haveCoral()) {
      armArbFF = ArmConstants.kArmKfCoral;
    } else {
      armArbFF = ArmConstants.kArmKfNoCoral;
    }

    inputs.encoderRotations = armEncoder.getPosition();

    if (ARM_STUCK_ERROR) {
      armTargetEncoderRotations = inputs.encoderRotations;
    }

    armClosedLoopController.setReference(
        armTargetEncoderRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        armArbFF * Math.cos(Math.toRadians(inputs.currentAngleDEG)));

    inputs.targetAngleDEG = armTargetDEG;
    inputs.targetEncoderRotations = this.armTargetEncoderRotations;
    inputs.encoderRotations = armEncoder.getPosition();
    inputs.armArbFF = armArbFF;
    inputs.ARM_STUCK_ERROR = this.ARM_STUCK_ERROR;
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
      armTargetDEG = initialDegree;
      armTargetEncoderRotations = armTargetDEG * ArmConstants.kArmGearing / 360.0;
      armEncoder.setPosition(armTargetEncoderRotations);
    } else {
      // Bad ThroughBore Encoder Reading
      TBE_Valid = false;
    }
  }

  @Override
  public void setArbFFConstant(double volts) {
    if (volts >= -0.4 && volts <= 0.4)
      armArbFF = volts;
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition) {
    if (TBE_Valid) {
      this.armTargetDEG = requestedPosition;
      this.armTargetEncoderRotations = this.armTargetDEG * ArmConstants.kArmGearing / 360.0;
    }
  }

  @Override
  public double getTargetAngleDEG() {
    return this.armTargetDEG;
  }

  @Override
  public double getCurrentArmDEG() {
    return armEncoder.getPosition() * 360.0 / ArmConstants.kArmGearing;
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    final SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop
        .pidf(kP, 0.0, kD, 0.0).maxMotion
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
}
