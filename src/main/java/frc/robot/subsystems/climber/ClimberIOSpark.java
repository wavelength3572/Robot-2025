package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;

public class ClimberIOSpark implements ClimberIO {
  private SparkMax climberMotor = new SparkMax(ClimberConstants.canId, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;

  public ClimberIOSpark() {
    climberMotor.configure(
        ClimberConfigs.ClimberSubsystem.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    climberMotor.set(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.currentClimbState = this.currentClimberState;
    switch (currentClimberState) {
      case STOWED:
        break;
      case DEPLOY:
        break;
      case CLIMB:
        break;
      case FINAL:
        break;
      default:
        break;
    }
  }

  public void deployClimber() {
    currentClimberState = CLIMB_STATE.DEPLOY;
    climberMotor.set(ClimberConstants.deployPower);
  }

  public void stowClimber() {}

  public void stopClimber() {
    climberMotor.set(0.0);
  }

  public void climb() {
    currentClimberState = CLIMB_STATE.DEPLOY;
    climberMotor.set(ClimberConstants.climbPower);
  }

  @Override
  public boolean isClimberDeployed() {
    return (this.currentClimberState != CLIMB_STATE.STOWED);
  }
}
