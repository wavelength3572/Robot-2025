package frc.robot.subsystems.coral.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {

  // The motor and encoder remain the same.
  private SparkFlex armMotor = new SparkFlex(ArmConstants.motorCanId, MotorType.kBrushless);
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Instead of storing the target in radians, store it in native units (motor rotations).
  private double armCurrentTargetNative = 0.0;

  // Simulation setup and variables
  private final DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkFlexSim armMotorSim;

  // Standard classes for controlling our arm.
  // Note: The PID controller now works in native units.
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ArmConstants.kArmKp,
          ArmConstants.kArmKi,
          ArmConstants.kArmKd,
          new TrapezoidProfile.Constraints(2.45, 2.45)); // units should be tuned to native units

  ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kArmkS, ArmConstants.kArmG, ArmConstants.kArmkV, ArmConstants.kArmkA);

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmConstants.kArmGearing,
          SingleJointedArmSim.estimateMOI(ArmConstants.kArmLengthMeters, ArmConstants.kArmMass),
          ArmConstants.kArmLengthMeters,
          Units.degreesToRadians(ArmConstants.kArmMinAngleDegrees),
          Units.degreesToRadians(ArmConstants.kArmMaxAngleDegrees),
          true,
          Units.degreesToRadians(ArmConstants.kArmHomeAngleDegrees),
          0.0,
          0.0);

  public ArmIOSim() {
    armMotor.configure(
        ArmConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(0);
    armMotorSim = new SparkFlexSim(armMotor, armMotorModel);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update simulation dynamics.
    m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(0.020);

    double armAngularVelocityRadPerSec = m_armSim.getVelocityRadPerSec(); // in rad/s
    double armRPS = (armAngularVelocityRadPerSec / (2.0 * Math.PI)) * ArmConstants.kArmGearing;
    double armRPM = armRPS * 60.0;

    armMotorSim.iterate(armRPM, RobotController.getBatteryVoltage(), 0.02);

    // **Manually update the encoder using the simulation's current angle:**
    double simulatedArmRadians = m_armSim.getAngleRads();
    double simulatedNative = (simulatedArmRadians / (2.0 * Math.PI)) * ArmConstants.kArmGearing;
    armEncoder.setPosition(simulatedNative);

    // Use native units for closed-loop control.
    m_controller.setGoal(armCurrentTargetNative);

    // Get current encoder reading in native units (motor rotations).
    double currentNative = armEncoder.getPosition();
    double pidOutput = m_controller.calculate(currentNative);
    double feedforwardOutput =
        m_feedforward.calculate(
            m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
    armMotor.set((pidOutput + feedforwardOutput) / RobotController.getBatteryVoltage());

    // For logging, convert native units (motor rotations) to physical arm angle in radians.
    double armAngleRadiansCalc = (currentNative / ArmConstants.kArmGearing) * (2.0 * Math.PI);

    inputs.setpoint = (armCurrentTargetNative / ArmConstants.kArmGearing) * (2.0 * Math.PI);
    inputs.encoderPositionRotations = armEncoder.getPosition();
    inputs.armAngleRad = simulatedArmRadians;
    inputs.armAngleRadCalc = armAngleRadiansCalc;
    inputs.motorVelocityRPM = armEncoder.getVelocity() / ArmConstants.kArmGearing;
    inputs.appliedVolts = armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = armMotorSim.getMotorCurrent();
    inputs.pidOutput = pidOutput;
    inputs.feedforwardOutput = feedforwardOutput;
    inputs.armAngleDegrees = Units.radiansToDegrees(simulatedArmRadians);
    inputs.armAngleDegreesCalc = Units.radiansToDegrees(armAngleRadiansCalc);
  }

  /**
   * Set the target arm angle in radians (physical units). Converts the requested angle to native
   * units (motor rotations) for closed-loop control.
   */
  @Override
  public void setAngleDegrees(double requestedAngleDegrees) {
    // Conversion: motor rotations = (requestedAngleRadians / (2π)) * gear ratio.
    armCurrentTargetNative =
        (Units.degreesToRadians(requestedAngleDegrees) / (2.0 * Math.PI))
            * ArmConstants.kArmGearing;
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // In simulation, brake mode might not be simulated.
  }

  @Override
  public double getAngleInRadians() {
    // Convert the current encoder reading (motor rotations) to physical arm angle in radians.
    double armRotations = armEncoder.getPosition() / ArmConstants.kArmGearing;
    return armRotations * (2.0 * Math.PI);
  }

  @Override
  public double getAngleInDegrees() {
    return Units.radiansToDegrees(getAngleInRadians());
  }
}
