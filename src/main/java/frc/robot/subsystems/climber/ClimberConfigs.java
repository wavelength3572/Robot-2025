package frc.robot.subsystems.climber;

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
          .voltageCompensation(12);
    }
  }
}
