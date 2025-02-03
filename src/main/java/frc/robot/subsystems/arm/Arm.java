package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final LoggedTunableNumber ArmkP =
      new LoggedTunableNumber("Arm/kEp", ArmConstants.kArmKp);
  private static final LoggedTunableNumber ArmkD =
      new LoggedTunableNumber("Arm/kEd", ArmConstants.kArmKd);
  private static final LoggedTunableNumber ArmkF =
      new LoggedTunableNumber("Arm/kEf", ArmConstants.kArmKf);

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (ArmkP.hasChanged(hashCode())
        || ArmkD.hasChanged(hashCode())
        || ArmkF.hasChanged(hashCode())) {
      io.setPIDValues(ArmkP.get(), ArmkD.get(), ArmkF.get());
    }
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setPositionDEG(Double requestedPosition) {
    io.setAngleDEG(requestedPosition);
  }

  public double getAngleDEG() {
    return io.getAngleDEG();
  }
}
