package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOSpark implements ClimberIO {

  private SparkMax climberMotor = new SparkMax(ClimberConstants.canId, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();

  private Relay spike = new Relay(0);

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;

  private double targetPosition = 0.0;

  private static final LoggedTunableNumber ClimberkP =
      new LoggedTunableNumber("Climber/kp", ClimberConstants.climberKp);

  public ClimberIOSpark() {
    climberMotor.configure(
        ClimberConfigs.ClimberSubsystem.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    climberMotor.set(0.0);
    climberEncoder.setPosition(0.0);
    spike.setDirection(Relay.Direction.kBoth);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (ClimberkP.hasChanged(hashCode())) {
      final SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pidf(ClimberkP.get(), 0.0, 0.0, 0.0);
      climberMotor.configure(
          config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    switch (currentClimberState) {
      case STOWED:
        climberMotor.set(0.0);
        // setRelayState(Relay.Value.kForward); //Foot shorter
        break;
      case DEPLOY:
        // climberMotor.set(ClimberConstants.deployPower);
        climberController.setReference(ClimberConstants.DEPLOY_POSITION, ControlType.kPosition);
        // setRelayState(Relay.Value.kForward); //Foot shorter
        break;
      case CLIMB:
        // climberMotor.set(ClimberConstants.climbPower);
        climberController.setReference(ClimberConstants.CLIMBED_POSITION, ControlType.kPosition);
        setRelayState(Relay.Value.kReverse); // Foot longer
        break;
      case FINAL:
        // climberMotor.set(0.0);
        climberController.setReference(targetPosition, ControlType.kPosition);
        break;
      default:
        break;
    }

    inputs.appliedVolts = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.currentPosition = climberEncoder.getPosition();
    inputs.targetPosition = this.targetPosition;
    inputs.currentClimbState = this.currentClimberState;
    inputs.relayState = spike.get();
  }

  public void deployClimber() {
    currentClimberState = CLIMB_STATE.DEPLOY;
  }

  public void stopClimber() {
    targetPosition = climberEncoder.getPosition();
    currentClimberState = CLIMB_STATE.FINAL;
  }

  public void climb() {
    if (currentClimberState == CLIMB_STATE.DEPLOY) {
      currentClimberState = CLIMB_STATE.CLIMB;
    }
  }

  public void setRelayState(Relay.Value newState) {
    spike.set(newState);
  }

  @Override
  public boolean isClimberDeployed() {
    return (this.currentClimberState != CLIMB_STATE.STOWED);
  }
}
