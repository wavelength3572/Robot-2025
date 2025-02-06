package frc.robot.subsystems.arm;

import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final LoggedTunableNumber ArmkP =
      new LoggedTunableNumber("Arm/kEp", ArmConstants.kArmKp);
  private static final LoggedTunableNumber ArmkD =
      new LoggedTunableNumber("Arm/kEd", ArmConstants.kArmKd);
  private static final LoggedTunableNumber ArmkF =
      new LoggedTunableNumber("Arm/kEf", ArmConstants.kArmKf);
  private static final LoggedTunableNumber ArmVel =
      new LoggedTunableNumber("Arm/kEVel", ArmConstants.kArmVel);
  private static final LoggedTunableNumber ArmAcc =
      new LoggedTunableNumber("Arm/kEAcc", ArmConstants.kArmAcc);

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    if (ArmkP.hasChanged(hashCode())
        || ArmkD.hasChanged(hashCode())
        || ArmkF.hasChanged(hashCode())
        || ArmVel.hasChanged(hashCode())
        || ArmAcc.hasChanged(hashCode())) {
      io.setPIDValues(ArmkP.get(), ArmkD.get(), ArmkF.get(), ArmVel.get(), ArmAcc.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    io.setPIDValues(ArmkP.get(), ArmkD.get(), preset.getArmFF(), ArmVel.get(), ArmAcc.get());
    setAngleDEG(preset.getArmAngle());
  }

  public void setAngleDEG(Double requestedPosition) {
    if (requestedPosition >= ArmConstants.armMinAngle
        && requestedPosition <= ArmConstants.armMaxAngle) io.setAngleDEG(requestedPosition);
  }

  public double getAngleDEG() {
    return io.getAngleDEG();
  }

  public double getSetpointDEG() {
    return inputs.targetAngleDEG;
  }

  public boolean isAtGoal() {
    return Math.abs(getAngleDEG() - getSetpointDEG()) < ArmConstants.kSetpointThresholdDEG;
  }
}
