package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  private boolean isDeployClimberTriggered = false;

  private static final LoggedTunableNumber AlgaekP =
      new LoggedTunableNumber("Algae/kEp", AlgaeConstants.kAlgaeDeployKp);
  private static final LoggedTunableNumber AlgaekD =
      new LoggedTunableNumber("Algae/kEd", AlgaeConstants.kAlgaeDeployKd);
  private static final LoggedTunableNumber AlgaeTargetAngle =
      new LoggedTunableNumber("Algae/AlgaeTargetPosition", AlgaeConstants.kAlgaeDeployInitalAngle);
  private static final LoggedTunableNumber AlgaeDeployVolts =
      new LoggedTunableNumber("Algae/AlgaeDeployVolts", 0.0);
  private static final LoggedTunableNumber AlgaeIntakeVolts =
      new LoggedTunableNumber("Algae/AlgaeDeployVolts", 0.0);

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public void periodic() {
    if (AlgaekP.hasChanged(hashCode()) || AlgaekD.hasChanged(hashCode())) {
      io.setPIDValues(AlgaekP.get(), AlgaekD.get());
    }

    if (AlgaeTargetAngle.hasChanged(hashCode())) {
      io.setDeployPositionAngle(AlgaeTargetAngle.get());
    }

    if (AlgaeDeployVolts.hasChanged(hashCode())) {
      io.setDeployVolts(AlgaeDeployVolts.get());
    }

    if (AlgaeIntakeVolts.hasChanged(hashCode())) {
      io.setDeployVolts(AlgaeIntakeVolts.get());
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
    if (!isDeployClimberTriggered) {
      io.pushAlgae();
    }
  }

  public void pullAlgae() {
    if (!isDeployClimberTriggered) {
      io.pullAlgae();
    }
  }

  public void stopAlgae() {
    io.stopAlgae();
  }

  public double getCurrentSpeedRPM() {
    return io.getCurrentSpeedRPM();
  }

  // Deploy Motor Methods
  public void deployAlgae() {
    if (!isDeployClimberTriggered) {
      io.deployAlgae();
    }
  }

  public void stowAlgae() {
    if (!isDeployClimberTriggered) {
      io.stowAlgae();
    }
  }

  public void setDeployPositionAngle(double angle) {
    if (!isDeployClimberTriggered) {
      if (angle >= AlgaeConstants.MIN_ANGLE && angle <= AlgaeConstants.MAX_ANGLE) {
        io.setDeployPositionAngle(angle);
      }
    }
  }

  public double getDeployPositionAngle() {
    return io.getDeployPositionAngle();
  }

  public void deployClimberTriggered() {
    deployAlgae(); // deploy the algae before we disable the ability to do so
    isDeployClimberTriggered = true;
  }
}
