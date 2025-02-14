package frc.robot.subsystems.algae;

public class AlgaeIOVirtualSim implements AlgaeIO {

  private double targetAngleDEG = AlgaeConstants.algaeStartAngle;
  private double targetEncoderRotations =
      AlgaeConstants.algaeStartAngle * AlgaeConstants.kAlgaeDeployGearing / 360.0;
  private double virtualEncoderRotations =
      AlgaeConstants.algaeStartAngle * AlgaeConstants.kAlgaeDeployGearing / 360.0;

  private double requestedSpeed = 0.0;
  private boolean haveAlgae = false;

  public AlgaeIOVirtualSim() {}

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Capture Motor Inputs (Simulating Intake/Outtake)
    inputs.captureRequestedSpeed = this.requestedSpeed;

    // Simulated Deploy Motor Movement
    inputs.targetAngleDEG = this.targetAngleDEG;
    inputs.currentAngleDEG = virtualEncoderRotations * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
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
  public void setTargetAngleDEG(double requestedPosition) {
    this.targetAngleDEG = requestedPosition;
    this.targetEncoderRotations = this.targetAngleDEG * AlgaeConstants.kAlgaeDeployGearing / 360.0;
  }

  @Override
  public double getTargetAngleDEG() {
    return this.targetAngleDEG;
  }

  @Override
  public double getCurrentAngleDEG() {
    return virtualEncoderRotations * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
  }

  @Override
  public void setAlgaeInRobot(boolean algaeInRobot) {
    haveAlgae = algaeInRobot;
  }

  @Override
  public boolean getAlgaeInRobot() {
    return haveAlgae;
  }

  @Override
  public double getCurrentSpeedRPM(){
    return requestedSpeed;
  }
}
