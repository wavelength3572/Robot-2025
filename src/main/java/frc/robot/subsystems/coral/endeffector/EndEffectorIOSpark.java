package frc.robot.subsystems.coral.endeffector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
// You'll need to create these classes similar to your elevator equivalents.

public class EndEffectorIOSpark implements EndEffectorIO {
  // Create a SparkMax instance for the end effector motor.
  private final SparkMax motor;
  // Optionally, if you need an encoder for sensor feedback:
  private final RelativeEncoder encoder;

  /**
   * Constructs the EndEffectorIOSpark. Replace EndEffectorConstants.motorCanId with the proper CAN
   * ID for your end effector.
   */
  public EndEffectorIOSpark() {
    // Initialize the motor using the specified CAN ID and MotorType.
    motor = new SparkMax(EndEffectorConstants.motorCanId, MotorType.kBrushless);
    // Configure the motor using your end effector's configuration.
    motor.configure(
        EndEffectorConfigs.Config, // similar to ElevatorConfigs but for the end effector
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Optionally, get the encoder if you want to log its position, even if you don't use
    // closed-loop control.
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // In a simple end effector, we may not have extensive sensor feedback.
    // Here we set the applied voltage based on the motor output and battery voltage.
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    // Use a placeholder calculation for current draw.
    inputs.currentAmps = Math.abs(motor.get()) * 10.0; // Adjust scaling as needed.
    // Set a nominal temperature.
    inputs.tempCelsius = 25.0;
    // Define the active state as whether a nonzero command is being sent.
    inputs.isActive = (motor.get() != 0.0);
  }

  @Override
  public void runOpenLoop(double output) {
    // Command the motor directly using percent output.
    motor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    // Convert the voltage command into a percent output.
    double batteryVoltage = RobotController.getBatteryVoltage();
    double percentOutput = volts / batteryVoltage;
    percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
    motor.set(percentOutput);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runSpeed(double speed) {
    // For now, treat runSpeed the same as open-loop control.
    runOpenLoop(speed);
  }

  @Override
  public void setActive(boolean active) {
    // If desired, store or act on the active state.
    // For many simple implementations, you might not need to do anything here.
  }
}
