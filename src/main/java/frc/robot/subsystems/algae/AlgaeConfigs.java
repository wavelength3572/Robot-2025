package frc.robot.subsystems.algae;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class AlgaeConfigs {

  public static final class AlgaeSubsystem {
    public static final SparkMaxConfig algaeCaptureConfig = new SparkMaxConfig();
    public static final SparkMaxConfig algaeDeployConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      algaeCaptureConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(AlgaeConstants.algaeCaptureCurrentLimit)
          .voltageCompensation(12);
      algaeCaptureConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(AlgaeConstants.kAlgaeCaptureKp)
          .d(AlgaeConstants.kAlgaeCaptureKd)
          .outputRange(-1, 1);

      algaeDeployConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(AlgaeConstants.algaeDeployCurrentLimit)
          .closedLoopRampRate(.25)
          .voltageCompensation(12);
      algaeDeployConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(AlgaeConstants.kAlgaeDeployKp)
          .d(AlgaeConstants.kAlgaeDeployKd)
          .outputRange(-1, 1);
    }
  }
}
