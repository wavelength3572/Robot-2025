package frc.robot.subsystems.coral.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A simulation implementation of the Arm I/O.
 * This class implements the ArmIO interface (defined as a nested interface in the Arm class)
 * and simulates the arm's rotation using closed-loop PID control. It uses rotations as its
 * native unit (1 rotation = 360°).
 */
public class ArmIOSim implements Arm.ArmIO {

  // Create a SparkMax instance to simulate the arm motor.
  private final SparkMax armMotor = new SparkMax(ArmConstants.motorCanId, MotorType.kBrushless);
  // Get the encoder from the motor (encoder units will be in rotations).
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  
  // The target for the arm, expressed in rotations.
  private double armTargetRotations = 0.0;
  
  // Simulation model for the arm motor. (For example, a NEO motor model.)
  private final DCMotor armMotorModel = DCMotor.getNEO(1); // Adjust as needed.
  private SparkMaxSim armMotorSim;
  
  
  // PID controller for the arm (units: rotations).
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          ArmConstants.kArmKp,
          ArmConstants.kArmKi,
          ArmConstants.kArmKd,
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxAngularVelocityRotations, 
              ArmConstants.kMaxAngularAccelerationRotations));

  public ArmIOSim() {
    // Configure the motor using your Arm configuration.
    armMotor.configure(
        ArmConfigs.ArmSubsystem, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);
    // Reset the encoder to 0.
    armEncoder.setPosition(0);
    
    // Create the simulation object for the SparkMax.
    armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
  }

  @Override
  public void updateInputs() {
    // Compute the voltage applied to the motor.
    double appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    double dt = 0.020; // 20 ms timestep.
    
    // Update the simulation with the current voltage input.
    armMotorSim.setInput(appliedVolts);
    armMotorSim.update(dt);

    // Update the encoder with the simulated position (in rotations).
    armEncoder.setPosition(armMotorSim.getPosition());
    
    // Update the PID controller: set the goal (in rotations) and calculate output.
    pidController.setGoal(armTargetRotations);
    double pidOutput = pidController.calculate(armEncoder.getPosition());
    
    // Command the motor with the PID output.
    // Convert the output (volts) to a percent output by dividing by battery voltage.
    double percentOutput = pidOutput / RobotController.getBatteryVoltage();
    armMotor.set(percentOutput);
  }

  @Override
  public double getAngle() {
    // Return the current arm angle in degrees (convert rotations to degrees).
    return armEncoder.getPosition() * 360.0;
  }

  @Override
  public void setAngle(double angleDegrees) {
    // Convert the desired angle (in degrees) to rotations.
    armTargetRotations = angleDegrees / 360.0;
  }
}
