package frc.robot.subsystems.algae;

public class AlgaeIOVirtualSim implements AlgaeIO {

  private double targetEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);
  private double virtualEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);

  private double requestedSpeed = 0.0;
  private boolean haveAlgae = false;
  private double appliedVoltage = 0.0;

  public AlgaeIOVirtualSim() {}

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // Update the basic angle and encoder values.
    inputs.currentAngle = virtualEncoderRotations * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
    // inputs.encoderRotations = virtualEncoderRotations;

    // Simulate the applied voltage (this is just stored for logging/simulation
    // purposes)
    inputs.deployAppliedVolts = appliedVoltage;

    // Simulate gradual movement toward the target (smooth virtual motion)
    if (virtualEncoderRotations < targetEncoderRotations) {
      virtualEncoderRotations = Math.min(virtualEncoderRotations + 0.2, targetEncoderRotations);
    } else if (virtualEncoderRotations > targetEncoderRotations) {
      virtualEncoderRotations = Math.max(virtualEncoderRotations - 0.2, targetEncoderRotations);
    }

    // Algae Detection (Simulating Intake State)
    inputs.haveAlgae = haveAlgae;
  }

  @Override
  public void setHaveAlgae(boolean haveAlgae) {
    this.haveAlgae = haveAlgae;
  }

  @Override
  public boolean haveAlgae() {
    return haveAlgae;
  }

  @Override
  public double getCurrentSpeedRPM() {
    return requestedSpeed;
  }

  @Override
  public double getDeployPositionAngle() {
    return virtualEncoderRotations * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
  }

  public double angleToRotations(double angle) {
    return (angle / 360.0) * AlgaeConstants.kAlgaeDeployGearing;
  }

  public double rotationsToAngle(double rotations) {
    return (rotations / AlgaeConstants.kAlgaeDeployGearing) * 360.0;
  }
}
