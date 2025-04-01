package frc.robot.subsystems.coral.elevator;

import static frc.robot.subsystems.coral.elevator.ElevatorConstants.leaderCanId;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.coral.arm.ArmConstants;

public final class ElevatorConfigs {

  public static final class ElevatorSubsystem {
    public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig followerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig newFollowerConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      leaderConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit)
          .voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      leaderConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(ElevatorConstants.kElevatorKp)
          .d(ElevatorConstants.kElevatorKd)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.kElevatorVel)
          .maxAcceleration(ElevatorConstants.kElevatorAcc)
          .allowedClosedLoopError(ArmConstants.kAllowableError);

      followerConfig
          // Configure basic settings of the elevator motor
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit)
          .voltageCompensation(12)
          .follow(leaderCanId, false);

      // Configure basic settings of the elevator motor
      newFollowerConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit)
          .voltageCompensation(12)
          .inverted(false);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      newFollowerConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(ElevatorConstants.kElevatorKp)
          .d(ElevatorConstants.kElevatorKd)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(ElevatorConstants.kElevatorVel)
          .maxAcceleration(ElevatorConstants.kElevatorAcc)
          .allowedClosedLoopError(ArmConstants.kAllowableError);
    }
  }
}
