package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;

public class ClimberIOSpark implements ClimberIO {
  private SparkMax climberMotor = new SparkMax(ClimberConstants.canId, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private boolean climberDeployed = false;

  private enum climberState {
    DEPLOYED,
    STOWED,
    CLIMB
  }

  private climberState currentClimberState = climberState.STOWED;

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
    inputs.climberDeployed = climberDeployed;
  }
}
