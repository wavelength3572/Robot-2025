package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ElevatorIOSim implements ElevatorIO {
  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2.45, 2.45));
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, .762, .762, 0.0);

  private double elevatorFFVolts = 0.0;
  private double elevatorAppliedVolts = 0.0;

  private double requestedPosition = 0.0;

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          10.0,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          0,
          2,
          true,
          0,
          0.01,
          0.0);
  private final Encoder m_encoder = new Encoder(0, 1);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  private DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNeoVortex(1), 0.025, ElevatorConstants.motorReduction),
          DCMotor.getNeoVortex(1));

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    m_elevatorSim.setInput(0.0);
    elevatorAppliedVolts =
        elevatorController.calculate(
            motorSim.getAngularPositionRotations(),
            this.requestedPosition * ElevatorConstants.motorReduction);
    motorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    inputs.setpoint = this.requestedPosition * ElevatorConstants.motorReduction;
    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = elevatorAppliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double requestedPosition) {
    this.requestedPosition = requestedPosition;
  }
}
