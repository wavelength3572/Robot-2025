package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  private boolean isDeployClimberTriggered = false;

  private static final LoggedTunableNumber AlgaeCapturekP =
      new LoggedTunableNumber("Algae/kACp", AlgaeConstants.kAlgaeCaptureKp);
  private static final LoggedTunableNumber AlgaeCapturekD =
      new LoggedTunableNumber("Algae/kACd", AlgaeConstants.kAlgaeCaptureKd);
  private static final LoggedTunableNumber AlgaeDeployVolts =
      new LoggedTunableNumber("Algae/AlgaeDeployVolts", 0.0);
  private static final LoggedTunableNumber AlgaeIntakeVolts =
      new LoggedTunableNumber("Algae/AlgaeIntakeVolts", 0.0);

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public void periodic() {

    // if (AlgaeCapturekP.hasChanged(hashCode()) || AlgaeCapturekD.hasChanged(hashCode())) {
    //   io.setCapturePIDValues(AlgaeCapturekP.get(), AlgaeCapturekD.get());
    // }

    // if (AlgaeDeployVolts.hasChanged(hashCode())) {
    //   io.setDeployVolts(AlgaeDeployVolts.get());
    // }

    // if (AlgaeIntakeVolts.hasChanged(hashCode())) {
    //   io.setIntakeVolts(AlgaeIntakeVolts.get());
    // }

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

  public void stowAlgae() {
    if (!isDeployClimberTriggered) {
      io.stowAlgae();
    }
  }

  public double getDeployPositionAngle() {
    return io.getDeployPositionAngle();
  }

  public void deployClimberTriggered() {
    // The climb switch was switched
    io.algaeInClimbPosition();
    isDeployClimberTriggered = true;
  }
}
