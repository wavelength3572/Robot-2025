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
          .openLoopRampRate(.5)
          .voltageCompensation(12);

      algaeDeployConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(AlgaeConstants.algaeDeployCurrentLimit)
          .voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      algaeDeployConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(AlgaeConstants.kAlgaeDeployKp)
          .d(AlgaeConstants.kAlgaeDeployKd)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(AlgaeConstants.kAlgaeDeployVel)
          .maxAcceleration(AlgaeConstants.kAlgaeDeployAcc)
          .allowedClosedLoopError(AlgaeConstants.kAlgaeDeployAllowableError);
    }
  }
}
