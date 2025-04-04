package frc.robot.subsystems.coral.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.coral.arm.ArmConstants;
import frc.robot.subsystems.coral.intake.IntakeConstants.INTAKE_STATE;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotStatus;

public class IntakeIOSpark implements IntakeIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the
  // elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.canId, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private AbsoluteEncoder armEncoder = intakeMotor.getAbsoluteEncoder();
  private Double requestedSpeed = 0.0;
  private boolean haveCoral = false;
  private double intakePushPower = IntakeConstants.intakeOutSpeed;
  private double coralDetectorCount = 0;

  private INTAKE_STATE currentIntakeState = INTAKE_STATE.OFF;

  public IntakeIOSpark() {
    intakeMotor.configure(
        IntakeConfigs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.set(requestedSpeed);
  }

  private static final LoggedTunableNumber PushPower =
      new LoggedTunableNumber("Intake/PushPower", IntakeConstants.intakeOutSpeed);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeState = currentIntakeState;
    if (PushPower.hasChanged(hashCode())) {
      intakePushPower = PushPower.get();
    }
    inputs.limitSwitch = intakeMotor.getForwardLimitSwitch().isPressed();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    if (currentIntakeState == INTAKE_STATE.PULL) {
      if ((intakeMotor.getOutputCurrent() >= 25.0 && inputs.appliedVolts > 0.0)) {
        coralDetectorCount++;
        if (coralDetectorCount >= 3) haveCoral = true;
      }
    } else {
      coralDetectorCount = 0;
      haveCoral = false;
    }

    if (RobotStatus.isArmInError() == false) {
      if (currentIntakeState == INTAKE_STATE.PULL && haveCoral) {
        intakeMotor.set(0.03);
      } else {
        intakeMotor.set(requestedSpeed);
      }
    } else {
      // Arm is in an emergency
      if (currentIntakeState == INTAKE_STATE.PUSH) {
        intakeMotor.set(requestedSpeed);
      } else {
        stopIntake();
      }
    }

    inputs.requestedSpeed = this.requestedSpeed;
    inputs.velocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
    inputs.motorPosition = intakeEncoder.getPosition();
    inputs.Arm_TBE = armEncoder.getPosition();
    inputs.Arm_TBE_DEG = ArmConstants.armTBEOffset + ((1.0 - inputs.Arm_TBE) * 120.0);
    inputs.haveCoral = haveCoral;
  }

  @Override
  public boolean haveCoral() {
    return haveCoral;
  }

  @Override
  public void pullCoral() {
    // haveCoral = false;
    currentIntakeState = INTAKE_STATE.PULL;
    this.requestedSpeed = IntakeConstants.intakeInSpeed;
  }

  @Override
  public void pushCoral() {
    currentIntakeState = INTAKE_STATE.PUSH;
    this.requestedSpeed = intakePushPower;
  }

  @Override
  public void stopIntake() {
    currentIntakeState = INTAKE_STATE.OFF;
    requestedSpeed = 0.0;
  }

  @Override
  public double get_Arm_TBE_DEG() {
    double Arm_TBE_Local = armEncoder.getPosition();
    double Arm_TBE_DEG_local = ArmConstants.armTBEOffset + ((1.0 - Arm_TBE_Local) * 120.0);
    return Arm_TBE_DEG_local;
  }

  @Override
  public void autoSetHaveCoral(boolean coral) {
    this.haveCoral = coral;
    currentIntakeState = INTAKE_STATE.PULL;
  }
}
