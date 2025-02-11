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

  public void pushCoral() {
    io.pushCoral();
  }

  public void pullCoral() {
    io.pullCoral();
  }

  public void stopIntake() {
    io.stopIntake();
  }

  public boolean getCoralInRobot() {
    return io.getCoralInRobot();
  }

  public void setCoralInRobot(boolean coralInRobot) {
    io.setCoralInRobot(coralInRobot);
  }
}
