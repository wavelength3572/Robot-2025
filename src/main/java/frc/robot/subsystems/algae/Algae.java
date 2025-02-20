package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  private static final LoggedTunableNumber AlgaekP =
      new LoggedTunableNumber("Algae/kEp", AlgaeConstants.kAlgaeDeployKp);
  private static final LoggedTunableNumber AlgaekD =
      new LoggedTunableNumber("Algae/kEd", AlgaeConstants.kAlgaeDeployKd);
  private static final LoggedTunableNumber AlgaeVel =
      new LoggedTunableNumber("Algae/kEVel", AlgaeConstants.kAlgaeDeployVel);
  private static final LoggedTunableNumber AlgaeAcc =
      new LoggedTunableNumber("Algae/kEAcc", AlgaeConstants.kAlgaeDeployAcc);

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public void periodic() {
    if (AlgaekP.hasChanged(hashCode())
        || AlgaekD.hasChanged(hashCode())
        || AlgaeVel.hasChanged(hashCode())
        || AlgaeAcc.hasChanged(hashCode())) {
      io.setPIDValues(AlgaekP.get(), AlgaekD.get(), AlgaeVel.get(), AlgaeAcc.get());
    }

    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public boolean isAlgaeInRobot() {
    return io.isAlgaeInRobot();
  }

  public void setAlgaeInRobot(boolean algaeInRobot) {
    io.setAlgaeInRobot(algaeInRobot);
  }

  // Capture Motor Methods
  public void pushAlgae() {
    io.pushAlgae();
  }

  public void pullAlgae() {
    io.pullAlgae();
  }

  public void stopAlgae() {
    io.stopAlgae();
  }

  public double getCurrentSpeedRPM() {
    return io.getCurrentSpeedRPM();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  // Deploy Motor Methods
  public void deployAlgae() {
    io.deployAlgae();
  }

  public void stowAlgae() {
    io.stowAlgae();
  }

  public void setDeployPositionAngle(double angle) {
    if (angle >= AlgaeConstants.MIN_ANGLE && angle <= AlgaeConstants.MAX_ANGLE) {
      io.setDeployPositionAngle(angle);
    }
  }

  public double getDeployPositionAngle() {
    return io.getDeployPositionAngle();
  }
}
