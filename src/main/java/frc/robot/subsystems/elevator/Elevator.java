package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber ElevatorkP =
      new LoggedTunableNumber("Elevator/kEp", ElevatorConstants.kElevatorKp);
  private static final LoggedTunableNumber ElevatorkD =
      new LoggedTunableNumber("Elevator/kEd", ElevatorConstants.kElevatorKd);
  private static final LoggedTunableNumber ElevatorkF =
      new LoggedTunableNumber("Elevator/kEf", ElevatorConstants.kElevatorKf);
  private static final LoggedTunableNumber ElevatorVel =
      new LoggedTunableNumber("Elevator/kEVel", ElevatorConstants.kElevatorVel);
  private static final LoggedTunableNumber ElevatorAcc =
      new LoggedTunableNumber("Elevator/kEAcc", ElevatorConstants.kElevatorAcc);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    if (ElevatorkP.hasChanged(hashCode())
        || ElevatorkD.hasChanged(hashCode())
        || ElevatorkF.hasChanged(hashCode())
        || ElevatorVel.hasChanged(hashCode())
        || ElevatorAcc.hasChanged(hashCode())) {
      io.setPIDValues(
          ElevatorkP.get(),
          ElevatorkD.get(),
          ElevatorkF.get(),
          ElevatorVel.get(),
          ElevatorAcc.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setPositionInches(Double requestedPosition) {
    this.setPositionMeters(Units.inchesToMeters(requestedPosition));
  }

  public void setPositionMeters(Double requestedPosition) {
    this.setPosition(
        ElevatorConstants.kElevatorGearing
            * (requestedPosition / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)));
  }

  public void setPosition(Double requestedPosition) {
    io.setPosition(requestedPosition);
  }

  public double getHeightInMeters() {
    return io.getHeightInMeters();
  }
}
