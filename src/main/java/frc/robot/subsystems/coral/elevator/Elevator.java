package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber ElevatorPosInches =
      new LoggedTunableNumber("Elevator/Goal", 0.0);
  private static final LoggedTunableNumber ElevatorkP =
      new LoggedTunableNumber("Elevator/kEp", ElevatorConstants.kElevatorKp);
  private static final LoggedTunableNumber ElevatorkD =
      new LoggedTunableNumber("Elevator/kEd", ElevatorConstants.kElevatorKd);
  // private static final LoggedTunableNumber ElevatorkF = new
  // LoggedTunableNumber("Elevator/kEf",
  // ElevatorConstants.kElevatorKf);
  private static final LoggedTunableNumber ElevatorVel =
      new LoggedTunableNumber("Elevator/kEVel", ElevatorConstants.kElevatorVel);
  private static final LoggedTunableNumber ElevatorAcc =
      new LoggedTunableNumber("Elevator/kEAcc", ElevatorConstants.kElevatorAcc);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    if (ElevatorkP.hasChanged(hashCode())
        || ElevatorkD.hasChanged(hashCode())
        || ElevatorVel.hasChanged(hashCode())
        || ElevatorAcc.hasChanged(hashCode())) {
      io.setPIDValues(ElevatorkP.get(), ElevatorkD.get(), ElevatorVel.get(), ElevatorAcc.get());
    }
    // if (ElevatorPosInches.hasChanged(hashCode()) ||
    // ElevatorkF.hasChanged(hashCode())) {
    // setPosition(ElevatorPosInches.get(), ElevatorkF.get());
    // }
    if (ElevatorPosInches.hasChanged(hashCode())) {
      setPositionInches(ElevatorPosInches.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    setPositionInches(preset.getElevatorHeight());
  }

  public void setPositionInches(Double requestedPosition) {
    // Must convert Inches to meters because kElevatorDrumRadius is in Meters
    double requestedPositionInMeters = Units.inchesToMeters(requestedPosition);

    // For the requested height in meters, calculate the motor rotation position
    double requestedPositionInRotations =
        ElevatorConstants.kElevatorGearing
            * (requestedPositionInMeters / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));

    setPosition(requestedPositionInRotations);
  }

  public void setPosition(Double requestedPosition) {
    // The requested position is in motor rotations
    // calulate the max motor rotation based on max elevator height
    // So we don't exceed it
    double maxRotations =
        ElevatorConstants.kElevatorGearing
            * (ElevatorConstants.kMaxElevatorHeightMeters
                / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    if (requestedPosition <= maxRotations) io.setPosition(requestedPosition);
  }

  public double getSetpointInInches() {
    return Units.metersToInches(getSetpointInMeters());
  }

  public double getSetpointInMeters() {
    return io.getSetpointInMeters();
  }

  public double getHeightInInches() {
    return Units.metersToInches(getHeightInMeters());
  }

  public double getHeightInMeters() {
    return io.getHeightInMeters();
  }

  public void recoverElevator() {
    io.recoverElevator();
  }

  public void clearElevatorError() {
    io.clearElevatorError();
  }

  public void runCharacterization(double output) {
    io.runCharacterization(output);
  }

  public double getFFCharacterizationVelocity() {
    return io.getFFCharacterizationVelocity();
  }

  public boolean isAtGoal() {
    return Math.abs(getHeightInInches() - getSetpointInInches())
        < ElevatorConstants.kSetpointThresholdINCHES;
  }

  public boolean isElevatorInError() {
    return io.isElevatorInError();
  }
}
