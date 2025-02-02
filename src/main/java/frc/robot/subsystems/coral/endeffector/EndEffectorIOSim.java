// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.coral.endeffector;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * A simple simulation implementation for the end effector. This class emulates a motor that either
 * intakes or ejects game pieces based on the applied voltage.
 */
public class EndEffectorIOSim implements EndEffectorIO {
  // Simulation state variables.
  private double appliedVolts = 0.0;
  // For a more realistic sim, you could add additional state variables such as simulated current.
  private double simulatedMotorOutput = 0.0;
  private static final double kDt = 0.020; // 20ms update period.
  private double lastTimestamp = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Calculate elapsed time.
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    if (dt <= 0) {
      dt = kDt;
    }
    lastTimestamp = now;

    // In this simple simulation, assume the motor output instantly reaches the commanded voltage.
    // (A more complex simulation might integrate dynamics over time.)
    simulatedMotorOutput = appliedVolts;

    // Update inputs. You might use RobotController.getBatteryVoltage() if you want to scale the
    // output.
    inputs.appliedVolts =
        simulatedMotorOutput; // or simulatedMotorOutput * RobotController.getBatteryVoltage();
    // Provide placeholder current draw and temperature values.
    inputs.currentAmps = Math.abs(simulatedMotorOutput) * 10.0; // placeholder calculation.
    inputs.tempCelsius = 25.0; // assume a nominal temperature.
    inputs.motorConnected = true;
  }

  @Override
  public void runOpenLoop(double output) {
    // Convert a percent output (-1.0 to 1.0) to voltage.
    appliedVolts = output * RobotController.getBatteryVoltage();
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  @Override
  public void runSpeed(double speed) {
    // For simulation, we treat runSpeed the same as runOpenLoop.
    runOpenLoop(speed);
  }
}
