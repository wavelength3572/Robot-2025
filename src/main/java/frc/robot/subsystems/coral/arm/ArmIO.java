// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.coral.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    /** Indicates if the arm motor is connected. */
    public boolean motorConnected = true;

    /** Current arm angle in radians. */
    public double angleRad = 0.0;

    /** Current angular velocity in radians per second. */
    public double angularVelocityRadPerSec = 0.0;

    /** The applied voltage to the arm motor. */
    public double appliedVolts = 0.0;

    /** The current drawn by the arm motor in Amps. */
    public double currentAmps = 0.0;

    /** Temperature of the motor in Celsius. */
    public double tempCelsius = 0.0;
  }

  /**
   * Update sensor inputs for the arm. This method should update the provided ArmIOInputs with the
   * latest sensor data.
   */
  default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Run the arm in open-loop control.
   *
   * @param output The percent output (-1.0 to 1.0) to apply to the motor.
   */
  default void runOpenLoop(double output) {}

  /**
   * Run the arm with a specified voltage.
   *
   * @param volts The voltage to apply to the arm motor.
   */
  default void runVolts(double volts) {}

  /** Stop the arm motor. */
  default void stop() {}

  /**
   * Run the arm to a target angle (in radians) using closed-loop control, with an additional
   * feedforward term.
   *
   * @param angleRad The desired arm angle in radians.
   * @param feedforward Additional voltage (or other feedforward signal) to aid in motion.
   */
  default void runPosition(double angleRad, double feedforward) {}

  /**
   * Set the PID constants for the arm's closed-loop controller.
   *
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   */
  default void setPID(double kP, double kI, double kD) {}

  /**
   * Enable or disable brake mode on the arm motor.
   *
   * @param enabled If true, brake mode is enabled.
   */
  default void setBrakeMode(boolean enabled) {}
}
