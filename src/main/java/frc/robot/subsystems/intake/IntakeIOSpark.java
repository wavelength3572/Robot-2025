package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSpark implements IntakeIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.canId, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private Double requestedSpeed = 0.0;

  public IntakeIOSpark() {
    intakeMotor.configure(
        IntakeConfigs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.set(requestedSpeed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.requestedSpeed = this.requestedSpeed;
    inputs.velocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setSpeed(double speed) {
    this.requestedSpeed = speed;
    intakeMotor.set(speed);
  }
}
