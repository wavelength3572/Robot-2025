package frc.robot.subsystems.elevator;

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
  private static final LoggedTunableNumber ElevatorkF =
      new LoggedTunableNumber("Elevator/kEf", ElevatorConstants.kElevatorKf);
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
      io.setPIDValues(
          ElevatorkP.get(), ElevatorkD.get(), 0.0, ElevatorVel.get(), ElevatorAcc.get());
    }
    if (ElevatorPosInches.hasChanged(hashCode()) || ElevatorkF.hasChanged(hashCode())) {
      setPosition(ElevatorPosInches.get(), ElevatorkF.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    // The preset Elevator Height is in inches but we set position in rotations but
    // based in meters
    double presetInMeters = Units.inchesToMeters(preset.getElevatorHeight());
    double setRotations =
        ElevatorConstants.kElevatorGearing
            * (presetInMeters / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    setPosition(setRotations, preset.getElevatorFF());
  }

  public void setPositionInches(Double requestedPosition) {
    double requestedPositionInMeters = Units.inchesToMeters(requestedPosition);

    double requestedPositionInRotations =
        ElevatorConstants.kElevatorGearing
            * (requestedPositionInMeters / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));

    io.setPosition(requestedPositionInRotations, ElevatorkF.get());
  }

  public void setPosition(Double requestedPosition, double requestedArbFF) {
    double maxRotations =
        ElevatorConstants.kElevatorGearing
            * (ElevatorConstants.kMaxElevatorHeightMeters
                / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    if (requestedPosition <= maxRotations) io.setPosition(requestedPosition, requestedArbFF);
  }

  public double getSetpointInInches() {
    return Units.metersToInches(getSetpointInMeters());
  }

  public double getSetpointInMeters() {
    return (inputs.setpoint * ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)
        / ElevatorConstants.kElevatorGearing;
  }

  public double getHeightInInches() {
    return Units.metersToInches(getHeightInMeters());
  }

  public double getHeightInMeters() {
    return io.getHeightInMeters();
  }

  public boolean isAtGoal() {
    return Math.abs(getHeightInInches() - getSetpointInInches())
        < ElevatorConstants.kSetpointThresholdINCHES;
  }
}
