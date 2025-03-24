package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotStatus;

public class ClimberIOSpark implements ClimberIO {

  private SparkMax climberMotor = new SparkMax(ClimberConstants.canId, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();

  private Relay spike = new Relay(0);

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;

  private double targetPosition = 0;

  private static final LoggedTunableNumber ClimberkP = new LoggedTunableNumber("Climber/kp",
      ClimberConstants.climberKp);

  private static final LoggedTunableNumber climbManualPosition = new LoggedTunableNumber("Climber/target",
      -1.0);

  private Servo servo = new Servo(1);

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

    inputs.climbingFinished = isClimbingFinished();

    // if (ClimberkP.hasChanged(hashCode())) {
    // final SparkMaxConfig config = new SparkMaxConfig();
    // config.closedLoop.pidf(ClimberkP.get(), 0.0, 0.0, 0.0);
    // climberMotor.configure(
    // config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // }

    if (climbManualPosition.hasChanged(hashCode())) {
      manualClimbPosition(climbManualPosition.get());
    }

    switch (currentClimberState) {
      case STOWED:
        climberMotor.set(0.0);
        break;
      case FAST_DEPLOY:
        targetPosition = drumToEncoder(ClimberConstants.FAST_DEPLOY_POSITION);
        if (RobotStatus.algaeArmIsSafeForClimbing()) {
          climberMotor.set(-1.0); // Deploy as fast as we can
          if (climberEncoder.getPosition() < targetPosition) {
            currentClimberState = CLIMB_STATE.DEPLOY;
          }
        }
        break;
      case DEPLOY:
        targetPosition = drumToEncoder(ClimberConstants.DEPLOY_POSITION);
        climberController.setReference(targetPosition, ControlType.kPosition);
        break;
      case CLIMB:
        targetPosition = drumToEncoder(ClimberConstants.CLIMBED_POSITION);
        climberController.setReference(targetPosition, ControlType.kPosition);
        setRelayState(Relay.Value.kReverse); // Foot longer
        break;
      case FINAL:
        if (RobotStatus.algaeArmIsSafeForClimbing()) {
          climberController.setReference(targetPosition, ControlType.kPosition);
        }
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
    currentClimberState = CLIMB_STATE.FAST_DEPLOY;
  }

  // public void stopClimber() {
  // targetPosition = climberEncoder.getPosition();
  // currentClimberState = CLIMB_STATE.FINAL;
  // }

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

  public void manualClimbPosition(double encoderRotations) {
    if (encoderRotations < -2) {
      targetPosition = encoderRotations;
      currentClimberState = CLIMB_STATE.FINAL;
    }
  }

  public double drumToEncoder(double drumRotations) {
    return drumRotations * ClimberConstants.kClimberGearing;
  }

  public double encoderToDrum(double encoderRotations) {
    return encoderRotations / ClimberConstants.kClimberGearing;
  }

  @Override
  public boolean isClimbingFinished() {
    if (currentClimberState == CLIMB_STATE.CLIMB) {
      double difference = Math.abs(drumToEncoder(ClimberConstants.CLIMBED_POSITION) - climberEncoder.getPosition());
      return (difference < ClimberConstants.CLIMBING_TOLERANCE) ? true : false;
    } else
      return false;
  }
}
