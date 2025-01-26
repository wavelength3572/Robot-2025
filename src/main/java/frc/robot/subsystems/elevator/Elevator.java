package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private boolean hasCoral = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
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
