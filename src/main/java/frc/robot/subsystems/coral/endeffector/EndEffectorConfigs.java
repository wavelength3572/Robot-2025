package frc.robot.subsystems.coral.endeffector;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * EndEffectorConfigs contains the SparkMax configuration for the end effector. This configuration
 * is similar in style to ElevatorConfigs but is tailored to the end effector’s needs.
 */
public final class EndEffectorConfigs {
  public static final SparkMaxConfig Config = new SparkMaxConfig();

  static {
    // Note: In this version of the REV API, SparkMaxConfig does not include an openLoopRampRate
    // field.
    // If you need to set an open-loop ramp rate for your end effector, call the method on the
    // SparkMax instance:
    // e.g., motor.setOpenLoopRampRate(0.2);

    // You can still configure other parameters here.
    // For example, to set the idle mode and current limit (if supported), you might do:
    // Config.idleMode = IdleMode.kCoast;
    // Config.currentLimit = 40;
  }

  private EndEffectorConfigs() {
    // Prevent instantiation.
  }
}
