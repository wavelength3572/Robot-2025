package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.leaderCanId;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ElevatorConfigs {

  public static final class ElevatorSubsystem {
    public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig followerConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      leaderConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.05)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      followerConfig
          // Configure basic settings of the elevator motor
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(12)
          .follow(leaderCanId);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */

    }
  }
}
