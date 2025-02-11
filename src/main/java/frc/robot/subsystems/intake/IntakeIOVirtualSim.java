package frc.robot.subsystems.intake;

public class IntakeIOVirtualSim implements IntakeIO {

  private Double requestedSpeed = 0.0;
  private boolean haveCoral = false;

  public IntakeIOVirtualSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.requestedSpeed = this.requestedSpeed;
  }

  @Override
  public void setSpeed(double speed) {
    this.requestedSpeed = speed;
  }

  @Override
  public void setCoralInRobot(boolean coralInRobot) {
    haveCoral = coralInRobot;
  }

  @Override
  public boolean getCoralInRobot() {
    return haveCoral;
  }
}
