package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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
        || ElevatorkF.hasChanged(hashCode())
        || ElevatorVel.hasChanged(hashCode())
        || ElevatorAcc.hasChanged(hashCode())) {
      io.setPIDValues(
          ElevatorkP.get(),
          ElevatorkD.get(),
          ElevatorkF.get(),
          ElevatorVel.get(),
          ElevatorAcc.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    boolean isAtGoal = isAtGoal();
    Logger.recordOutput("Elevator/AtGoal", isAtGoal);
  }

  public void setPositionInches(Double requestedPosition) {
    this.setPositionMeters(Units.inchesToMeters(requestedPosition));
  }

  public void setPositionMeters(Double requestedPosition) {
    this.setPosition(
        ElevatorConstants.kElevatorGearing
            * (requestedPosition / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)));
  }

  public void setPosition(Double requestedPosition) {
    if (requestedPosition
        <= ElevatorConstants.kElevatorGearing
            * (ElevatorConstants.kMaxElevatorHeightMeters
                / (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)))
      io.setPosition(requestedPosition);
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

  public Pose3d getElevator3DPose() {
    // update dynamic poses for 3D visualization.
    double elevatorOffset = this.getHeightInMeters();
    return new Pose3d(0, -0.0908, elevatorOffset + 0.24, new Rotation3d(0, 0, 0));
  }

  public boolean isAtGoal() {
    return Math.abs(getHeightInInches() - getSetpointInInches())
        < ElevatorConstants.kSetpointThresholdINCHES;
  }
}
