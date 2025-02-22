package frc.robot.subsystems.coral.arm;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ArmIOVirtualSim implements ArmIO {
  // Current target in degrees and corresponding encoder rotations.
  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations =
      ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;
  // Virtual encoder position (in rotations) used for simulation.
  private double armVirtualEncoder = ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  // Additional simulation variables
  private double appliedVoltage = 0.0;
  private double kP = 0.0;
  private double kD = 0.0;
  private double velocityMax = 0.0;
  private double accelerationMax = 0.0;

  public ArmIOVirtualSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update the basic angle and encoder values.
    inputs.targetAngleDEG = this.armTargetDEG;
    inputs.currentAngleDEG = armVirtualEncoder * 360.0 / ArmConstants.kArmGearing;
    inputs.targetEncoderRotations = armTargetEncoderRotations;
    inputs.encoderRotations = armVirtualEncoder;

    // Simulate the applied voltage (this is just stored for logging/simulation purposes)
    inputs.appliedVolts = appliedVoltage;

    // Here you could simulate velocity and current draw if desired.
    // For now we leave these as zero or default.
    inputs.velocityRPM = 0.0;
    inputs.currentAmps = 0.0;
    inputs.armArbFF = 0.0;
    inputs.armArbFF_COS = 0.0;
    inputs.TBE_Valid = true; // For simulation, we assume the sensor is always valid.

    // Simple simulation: slowly update the virtual encoder to reach the target.
    if (armVirtualEncoder < armTargetEncoderRotations) {
      if (armTargetEncoderRotations - armVirtualEncoder > 0.2) {
        armVirtualEncoder += 0.2;
      } else {
        armVirtualEncoder = armTargetEncoderRotations;
      }
    } else if (armVirtualEncoder > armTargetEncoderRotations) {
      if (armVirtualEncoder - armTargetEncoderRotations > 0.2) {
        armVirtualEncoder -= 0.2;
      } else {
        armVirtualEncoder = armTargetEncoderRotations;
      }
    }
  }

  @Override
  public void setVoltage(double volts) {
    // In simulation, we simply store the applied voltage.
    // More sophisticated simulation might update armVirtualEncoder based on volts.
    this.appliedVoltage = volts;
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition) {
    this.armTargetDEG = requestedPosition- 2.4; //3.2 is the fudge factor because when we request 45 the through bore actually reads 42.6
    this.armTargetEncoderRotations = this.armTargetDEG * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public double getTargetAngleDEG() {
    return this.armTargetDEG;
  }

  @Override
  public double getCurrentArmDEG() {
    return this.armVirtualEncoder * 360.0 / ArmConstants.kArmGearing;
  }

  @Override
  public void setInitialAngle(double initialDegree) {
    // Set both the virtual encoder and target to the initial angle.
    this.armVirtualEncoder = initialDegree * ArmConstants.kArmGearing / 360.0;
    this.armTargetDEG = initialDegree;
    this.armTargetEncoderRotations = initialDegree * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    // Store the PID (and motion) parameters for logging or future use.
    this.kP = kP;
    this.kD = kD;
    this.velocityMax = VelocityMax;
    this.accelerationMax = AccelerationMax;
  }
}
