package frc.robot.subsystems.arm;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ArmIOVirtualSim implements ArmIO {
  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations =
      ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;
  private double armVirtualEncoder = ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  // Simulation setup and variables

  public ArmIOVirtualSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.targetAngleDEG = this.armTargetDEG;
    inputs.currentAngleDEG = armVirtualEncoder * 360.0 / ArmConstants.kArmGearing;
    inputs.targetEncoderRotations = armTargetEncoderRotations;
    inputs.encoderRotations = armVirtualEncoder;

    if (armVirtualEncoder < armTargetEncoderRotations) {
      if (armTargetEncoderRotations - armVirtualEncoder > 0.2) armVirtualEncoder += 0.2;
      else armVirtualEncoder = armTargetEncoderRotations;
    } else if (armVirtualEncoder > armTargetEncoderRotations) {
      if (armVirtualEncoder - armTargetEncoderRotations > 0.2) armVirtualEncoder -= 0.2;
      else armVirtualEncoder = armTargetEncoderRotations;
    }
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition, double requestedArbFF) {
    this.armTargetDEG = requestedPosition;
    this.armTargetEncoderRotations = this.armTargetDEG * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public double getTargetAngleDEG() {
    return this.armTargetDEG;
  }

  @Override
  public double getCurrentArmDEG() {
    return this.armVirtualEncoder * (1.0 / ArmConstants.kArmGearing) * 360.0;
  }
}
