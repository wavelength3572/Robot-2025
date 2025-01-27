package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

// See
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
// and
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html

public class ElevatorIOPPCSim implements ElevatorIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(ElevatorConstants.leaderCanId, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private double elevatorCurrentTarget = 0.0;

  // Simulation setup and variables
  private final DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private SparkMaxSim elevatorMotorSim;

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ElevatorConstants.kElevatorKp,
          ElevatorConstants.kElevatorKi,
          ElevatorConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          ElevatorConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  public ElevatorIOPPCSim() {
    elevatorMotor.configure(
        ElevatorConfigs.ElevatorSubsystem.leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorEncoder.setPosition(0);
    elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * ElevatorConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);

    m_controller.setGoal(elevatorCurrentTarget);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(elevatorEncoder.getPosition());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    elevatorMotor.set((pidOutput + feedforwardOutput) / 12.0);
    // elevatorMotorSim.setAppliedOutput((pidOutput + feedforwardOutput) / 12.0);

    inputs.setpoint = this.elevatorCurrentTarget;
    inputs.positionRotations = elevatorEncoder.getPosition();
    inputs.elevatorHeight = m_elevatorSim.getPositionMeters();
    inputs.elevatorHeightCalc =
        (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
            * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
    inputs.appliedVolts = elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = elevatorMotorSim.getMotorCurrent();
    inputs.pidOutput = pidOutput;
    inputs.feedforwardOutput = feedforwardOutput;
  }

  @Override
  public void setPosition(double requestedPosition) {
    this.elevatorCurrentTarget = requestedPosition;
  }

  @Override
  public double getHeightInMeters() {
    return (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
        * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
  }
}
