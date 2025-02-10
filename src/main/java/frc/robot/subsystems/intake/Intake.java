package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class Intake {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setSpeed(Double requestedSpeed) {
    io.setSpeed(requestedSpeed);
  }

  public boolean getCoralInRobot() {
    return io.getCoralInRobot();
  }

  public void setCoralInRobot(boolean coralInRobot) {}
}
