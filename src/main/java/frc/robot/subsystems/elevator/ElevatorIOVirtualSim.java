package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ElevatorIOVirtualSim implements ElevatorIO {
  private double elevatorCurrentTarget = 0.0;
  private double elevatorVirtualEncoder = 0.0;

  // Simulation setup and variables

  public ElevatorIOVirtualSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.setpoint = this.elevatorCurrentTarget;
    inputs.positionRotations = elevatorVirtualEncoder;
    inputs.elevatorHeightCalc =
        Units.metersToInches(
            (elevatorVirtualEncoder / ElevatorConstants.kElevatorGearing)
                * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));

    if (elevatorVirtualEncoder < elevatorCurrentTarget) {
      if (elevatorCurrentTarget - elevatorVirtualEncoder > 1.0) elevatorVirtualEncoder += 1.0;
      else elevatorVirtualEncoder = elevatorCurrentTarget;
    } else if (elevatorVirtualEncoder > elevatorCurrentTarget) {
      if (elevatorVirtualEncoder - elevatorCurrentTarget > 1.0) elevatorVirtualEncoder -= 1.0;
      else elevatorVirtualEncoder = elevatorCurrentTarget;
    }
  }

  @Override
  public void setPosition(double requestedPosition) {
    this.elevatorCurrentTarget = requestedPosition;
  }

  @Override
  public double getHeightInMeters() {
    return (elevatorVirtualEncoder / ElevatorConstants.kElevatorGearing)
        * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
  }
}
