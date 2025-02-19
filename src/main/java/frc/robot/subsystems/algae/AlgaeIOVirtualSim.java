package frc.robot.subsystems.algae;

public class AlgaeIOVirtualSim implements AlgaeIO {

  private double targetEncoderRotations = 0.0;
  private double virtualEncoderRotations = 0.0;

  private double requestedSpeed = 0.0;
  private boolean haveAlgae = false;

  public AlgaeIOVirtualSim() {}

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Capture Motor Inputs (Simulating Intake/Outtake)
    inputs.captureRequestedSpeed = this.requestedSpeed;

    // Simulated Deploy Motor Movement
    inputs.targetEncoderRotations = this.targetEncoderRotations;
    inputs.encoderRotations = virtualEncoderRotations;

    // Simulate gradual movement toward the target (smooth virtual motion)
    if (virtualEncoderRotations < targetEncoderRotations) {
      virtualEncoderRotations = Math.min(virtualEncoderRotations + 0.2, targetEncoderRotations);
    } else if (virtualEncoderRotations > targetEncoderRotations) {
      virtualEncoderRotations = Math.max(virtualEncoderRotations - 0.2, targetEncoderRotations);
    }

    // Algae Detection (Simulating Intake State)
    inputs.algaeInRobot = haveAlgae;
  }

  @Override
  public void setSpeed(double speed) {
    this.requestedSpeed = speed;
  }

  @Override
  public void setAlgaeInRobot(boolean algaeInRobot) {
    haveAlgae = algaeInRobot;
  }

  @Override
  public boolean isAlgaeInRobot() {
    return haveAlgae;
  }

  @Override
  public double getCurrentSpeedRPM() {
    return requestedSpeed;
  }

  @Override
  public void deployAlgae() {
    targetEncoderRotations = AlgaeConstants.algaeDeployPosition;
  }

  @Override
  public void stowAlgae() {
    targetEncoderRotations = AlgaeConstants.algaeStowPosition;
  }

  @Override
  public void setDeployPositionAngle(double angle) {
    targetEncoderRotations = angle;
  }

  @Override
  public double getDeployPositionAngle() {
    return 0.0;
  }
}
