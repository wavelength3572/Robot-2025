package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ClimberConfigs {

  public static final class ClimberSubsystem {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      climberConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ClimberConstants.climberCurrentLimit)
          .openLoopRampRate(.1)
          // .closedLoopRampRate(.25)
          .voltageCompensation(12);
      climberConfig
          .closedLoop
          // Set PID values for position control
          .p(ClimberConstants.climberKp)
          .d(ClimberConstants.climberKd)
          .outputRange(-1.0, 1.0)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
  }
}
