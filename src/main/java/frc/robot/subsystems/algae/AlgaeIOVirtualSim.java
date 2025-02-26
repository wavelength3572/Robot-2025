package frc.robot.subsystems.algae;

import frc.robot.subsystems.coral.arm.ArmConstants;

public class AlgaeIOVirtualSim implements AlgaeIO {

  private double targetEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);
  private double virtualEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);

  private double requestedSpeed = 0.0;
  private boolean haveAlgae = false;
  private double algaeTargetDEG = AlgaeConstants.algaeStowPosition;
  private double appliedVoltage = 0.0;

  public AlgaeIOVirtualSim() {}

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // Update the basic angle and encoder values.
    inputs.targetAngle = this.algaeTargetDEG;
    inputs.currentAngle = virtualEncoderRotations * 360.0 / AlgaeConstants.kAlgaeDeployGearing;
    inputs.targetEncoderRotations = targetEncoderRotations;
    // inputs.encoderRotations = virtualEncoderRotations;

    // Simulate the applied voltage (this is just stored for logging/simulation
    // purposes)
    inputs.deployAppliedVolts = appliedVoltage;

    // Capture Motor Inputs (Simulating Intake/Outtake)
    inputs.captureRequestedSpeed = this.requestedSpeed;

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
    this.targetEncoderRotations = angleToRotations(AlgaeConstants.algaeDeployPosition);
  }

  @Override
  public void stowAlgae() {
    targetEncoderRotations = angleToRotations(AlgaeConstants.algaeStowPosition);
  }

  @Override
  public void setDeployPositionAngle(double angle) {
    this.algaeTargetDEG = angle;
    this.targetEncoderRotations = this.algaeTargetDEG * ArmConstants.kArmGearing / 360.0;
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
