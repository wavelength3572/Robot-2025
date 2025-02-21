package frc.robot.subsystems.coral.arm;

import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final LoggedTunableNumber ArmPosDegrees =
      new LoggedTunableNumber("Arm/Goal (DEG)", 0.0);
  private static final LoggedTunableNumber ArmkP =
      new LoggedTunableNumber("Arm/kEp", ArmConstants.kArmKp);
  private static final LoggedTunableNumber ArmkD =
      new LoggedTunableNumber("Arm/kEd", ArmConstants.kArmKd);
  // private static final LoggedTunableNumber ArmkF =
  //     new LoggedTunableNumber("Arm/kEf", ArmConstants.kArmKf);
  private static final LoggedTunableNumber ArmVel =
      new LoggedTunableNumber("Arm/kEVel", ArmConstants.kArmVel);
  private static final LoggedTunableNumber ArmAcc =
      new LoggedTunableNumber("Arm/kEAcc", ArmConstants.kArmAcc);

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setInitialAngle(double initialDegree) {
    io.setInitialAngle(initialDegree);
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    setAngleDEG(preset.getArmAngle());
  }

  public void setAngleDEG(Double requestedPosition) {
    if (requestedPosition >= ArmConstants.armMinAngle
        && requestedPosition <= ArmConstants.armMaxAngle) io.setTargetAngleDEG(requestedPosition);
  }

  public double getCurrentAngleDEG() {
    return io.getCurrentArmDEG();
  }

  public double getTargetAngleDEG() {
    return io.getTargetAngleDEG();
  }

  public boolean isAtGoal() {
    return Math.abs(getCurrentAngleDEG() - getTargetAngleDEG())
        < ArmConstants.kSetpointThresholdDEG;
  }
}
