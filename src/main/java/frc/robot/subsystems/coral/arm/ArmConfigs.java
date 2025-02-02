package frc.robot.subsystems.coral.arm;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ArmConfigs {
  public static final SparkMaxConfig armConfig = new SparkMaxConfig();

  static {
    // Configure basic settings of the arm motor
    armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.armCurrentLimit)
        .voltageCompensation(12);

    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(ArmConstants.kArmKp)
        .d(ArmConstants.kArmKd)
        .outputRange(-1, 1)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(ArmConstants.kArmVel)
        .maxAcceleration(ArmConstants.kArmAcc)
        .allowedClosedLoopError(0.1);
  }

  private ArmConfigs() {}
}
