package frc.robot.subsystems.coral.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOSpark implements ElevatorIO {

  // Initialize elevator SPARK. We will use MAXMotion position control for the
  // elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax leaderMotor = new SparkMax(ElevatorConstants.leaderCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      leaderMotor.getClosedLoopController();
  private RelativeEncoder leaderEncoder = leaderMotor.getEncoder();

  private SparkMax followerMotor =
      new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless);
  private RelativeEncoder followerEncoder = followerMotor.getEncoder();

  private double elevatorCurrentTarget = 0.0;
  private double elevatorCurrentArbFF = 0.0;

  public ElevatorIOSpark() {
    leaderMotor.configure(
        ElevatorConfigs.ElevatorSubsystem.leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    leaderEncoder.setPosition(0);

    followerMotor.configure(
        ElevatorConfigs.ElevatorSubsystem.followerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    followerEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        elevatorCurrentArbFF);

    inputs.setpoint = this.elevatorCurrentTarget;
    inputs.positionRotations = leaderEncoder.getPosition();
    inputs.velocityRPM = leaderEncoder.getVelocity();
    inputs.elevatorHeightCalc =
        Units.metersToInches(
            (leaderEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
                * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    inputs.appliedVolts = leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = leaderMotor.getOutputCurrent();
    inputs.feedforwardOutput = elevatorCurrentArbFF;

    inputs.followerPositionRotations = followerEncoder.getPosition();
    inputs.followerVelocityRPM = followerEncoder.getVelocity();
  }

  @Override
  public void setPosition(double requestedPosition) {
    if (requestedPosition <= .01) {
      this.elevatorCurrentArbFF = 0.0;
    } else {
      this.elevatorCurrentArbFF = ElevatorConstants.kElevatorKf;
    }
    this.elevatorCurrentTarget = requestedPosition;
  }

  @Override
  public double getHeightInMeters() {
    return (leaderEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
        * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
  }

  @Override
  public double getSetpointInMeters() {
    return (this.elevatorCurrentTarget * ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)
        / ElevatorConstants.kElevatorGearing;
  }

  @Override
  public void setPIDValues(double kP, double kD, double VelocityMax, double AccelerationMax) {
    final SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pidf(kP, 0.0, kD, 0.0)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(VelocityMax)
        .maxAcceleration(AccelerationMax)
        .allowedClosedLoopError(0.1);
    leaderMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
