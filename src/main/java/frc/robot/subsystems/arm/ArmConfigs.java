package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ArmConfigs {

  public static final class ArmSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      armConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ArmConstants.armCurrentLimit)
          .voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(ArmConstants.kArmKp)
          .d(ArmConstants.kArmKd)
          .outputRange(-1, 1);
    }
  }
}
