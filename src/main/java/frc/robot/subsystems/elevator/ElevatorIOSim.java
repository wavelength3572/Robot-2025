package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private PIDController elevatorController = new PIDController(0, 0, 0);

  private double elevatorFFVolts = 0.0;
  private double elevatorAppliedVolts = 0.0;

  private double requestedPosition = 0.0;

  private DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNeoVortex(1), 0.025, ElevatorConstants.motorReduction),
          DCMotor.getNeoVortex(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.setpoint = this.requestedPosition;
    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double requestedPosition) {
    this.requestedPosition = requestedPosition;
  }
}
