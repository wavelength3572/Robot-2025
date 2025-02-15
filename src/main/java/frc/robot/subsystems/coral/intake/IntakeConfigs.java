package frc.robot.subsystems.coral.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class IntakeConfigs {

  public static final class IntakeSubsystem {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(IntakeConstants.intakeCurrentLimit)
          .voltageCompensation(12);
    }
  }
}
