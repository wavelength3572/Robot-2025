package frc.robot.subsystems.arm;

import frc.robot.subsystems.elevator.ElevatorConstants;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ArmIOVirtualSim implements ArmIO {
  private double armTargetDEG = 0.0;
  private double armTargetEncoderRotations = 0.0;
  private double armVirtualEncoder = 0.0;

  // Simulation setup and variables

  public ArmIOVirtualSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.targetAngleDEG = this.armTargetDEG;
    inputs.currentAngleDEG =
        (armVirtualEncoder / ElevatorConstants.kElevatorGearing)
            * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
    inputs.targetEncoderRotations = armTargetEncoderRotations;
    inputs.encoderRotations = armVirtualEncoder;

    if (armVirtualEncoder < armTargetEncoderRotations) {
      if (armTargetEncoderRotations - armVirtualEncoder > 1.0) armVirtualEncoder += 1.0;
      else armVirtualEncoder = armTargetEncoderRotations;
    } else if (armVirtualEncoder > armTargetEncoderRotations) {
      if (armVirtualEncoder - armTargetEncoderRotations > 1.0) armVirtualEncoder -= 1.0;
      else armVirtualEncoder = armTargetEncoderRotations;
    }
  }

  @Override
  public void setAngleDEG(double requestedPosition) {
    this.armTargetDEG = requestedPosition;
    this.armTargetEncoderRotations = this.armTargetDEG * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public double getAngleDEG() {
    return this.armTargetEncoderRotations * (1.0 / ArmConstants.kArmGearing) * 360.0;
  }
}
