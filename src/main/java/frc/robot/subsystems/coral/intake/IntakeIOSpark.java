package frc.robot.subsystems.coral.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.coral.arm.ArmConstants;

public class IntakeIOSpark implements IntakeIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.canId, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private AbsoluteEncoder armEncoder = intakeMotor.getAbsoluteEncoder();
  private Double requestedSpeed = 0.0;
  private boolean haveCoral = false;
  private int currentSpikeCounter = 0;

  private enum intakeState {
    OFF,
    PUSH,
    PULL
  }

  private intakeState currentIntakeState = intakeState.OFF;

  public IntakeIOSpark() {
    intakeMotor.configure(
        IntakeConfigs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.set(requestedSpeed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // haveCoral = intakeMotor.getForwardLimitSwitch().isPressed();
    if (currentIntakeState == intakeState.PULL) {
      if (intakeMotor.getOutputCurrent() >= 45.0) {
        if (currentSpikeCounter > 2) {
          haveCoral = (haveCoral || intakeMotor.getOutputCurrent() >= 45.0);
        } else {
          currentSpikeCounter++;
        }
      }
    } else {
      haveCoral = false;
      currentSpikeCounter = 0;
    }

    if (currentIntakeState == intakeState.PULL && haveCoral) {
      intakeMotor.set(0.03);
    } else {
      intakeMotor.set(requestedSpeed);
    }

    inputs.requestedSpeed = this.requestedSpeed;
    inputs.velocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
    inputs.motorPosition = intakeEncoder.getPosition();
    inputs.Arm_TBE = armEncoder.getPosition();
    inputs.Arm_TBE_DEG = ArmConstants.armTBEOffset + ((1.0 - inputs.Arm_TBE) * 120.0);
    inputs.coralInRobot = haveCoral;
  }

  @Override
  public void setSpeed(double speed) {
    this.requestedSpeed = speed;
  }

  @Override
  public boolean getCoralInRobot() {
    return haveCoral;
  }

  @Override
  public void pullCoral() {
    currentIntakeState = intakeState.PULL;
    setSpeed(IntakeConstants.intakeInSpeed);
  }

  @Override
  public void pushCoral() {
    currentIntakeState = intakeState.PUSH;
    setSpeed(IntakeConstants.intakeOutSpeed);
  }

  @Override
  public void stopIntake() {
    currentIntakeState = intakeState.OFF;
    setSpeed(0.0);
  }

  @Override
  public double get_Arm_TBE_DEG() {
    double Arm_TBE_Local = armEncoder.getPosition();
    double Arm_TBE_DEG_local = ArmConstants.armTBEOffset + ((1.0 - Arm_TBE_Local) * 120.0);
    return Arm_TBE_DEG_local;
  }
}
