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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotStatus;

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

  private boolean firstTimeArmInError = true;
  private boolean inElevatorRecoveryMode = false;

  private static final LoggedTunableNumber ElevatorKg = new LoggedTunableNumber("Elevator/Kg", 0.0);

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
    inputs.leaderPositionRotations = leaderEncoder.getPosition();
    inputs.followerPositionRotations = followerEncoder.getPosition();
    inputs.followerError = inputs.leaderPositionRotations - inputs.followerPositionRotations;

    // If the arm is in error then don't move the elevator
    if (RobotStatus.isArmInError()) {
      if (firstTimeArmInError == true) {
        leaderMotor.set(0);
        firstTimeArmInError = false;
      }
      if (inElevatorRecoveryMode == false) {
        elevatorCurrentTarget = inputs.leaderPositionRotations;
      }
    }

    if (ElevatorKg.hasChanged(hashCode())) {
      runCharacterization(ElevatorKg.get());
    }

    // Comment out when running characterization
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        elevatorCurrentArbFF);

    inputs.setpoint = this.elevatorCurrentTarget;
    inputs.leaderVelocityRPM = leaderEncoder.getVelocity();
    inputs.followerVelocityRPM = followerEncoder.getVelocity();
    inputs.elevatorHeightCalc =
        Units.metersToInches(
            (leaderEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
                * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    inputs.leaderAppliedVolts =
        leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.followerAppliedVolts =
        followerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.leaderCurrentAmps = leaderMotor.getOutputCurrent();
    inputs.followerCurrentAmps = followerMotor.getOutputCurrent();
    inputs.feedforwardOutput = elevatorCurrentArbFF;
  }

  @Override
  public void setPosition(double requestedPosition) {
    // requestedPosition is in motor rotations
    if (RobotStatus.isArmInError() == false) {
      if (requestedPosition <= .01) {
        this.elevatorCurrentArbFF = 0.0;
      } else {
        this.elevatorCurrentArbFF = ElevatorConstants.kElevatorKf;
      }
      this.elevatorCurrentTarget = requestedPosition;
    }
  }

  @Override
  public void recoverElevator() {
    inElevatorRecoveryMode = true;
    this.elevatorCurrentArbFF = 0.0;
    this.elevatorCurrentTarget = 0.0;
  }

  @Override
  public void clearElevatorError() {
    firstTimeArmInError = true;
    inElevatorRecoveryMode = false;
  }

  @Override
  public void runCharacterization(double output) {
    leaderMotor.setVoltage(output);
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return leaderEncoder.getVelocity();
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
