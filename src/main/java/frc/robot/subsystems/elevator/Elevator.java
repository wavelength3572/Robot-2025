package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private boolean hasCoral = false;

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

  @Override
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
    Logger.recordOutput("Alignment/hasCoral", hasCoral());
  }

  public void setPosition(Double requestedPosition) {
    io.setPosition(requestedPosition);
  }

  public double getHeightInMeters() {
    return io.getHeightInMeters();
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void setHasCoral() {
    this.hasCoral = true;
  }

  public void clearHasCoral() {
    this.hasCoral = false;
  }
}
