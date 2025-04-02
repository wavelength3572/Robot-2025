package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  private boolean isDeployClimberTriggered = false;

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public void periodic() {
    // if (AlgaeDeployVolts.hasChanged(hashCode())) {
    //   io.setDeployVolts(AlgaeDeployVolts.get());
    // }

    // if (AlgaeIntakeVolts.hasChanged(hashCode())) {
    //   io.setIntakeVolts(AlgaeIntakeVolts.get());
    // }

    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public boolean haveAlgae() {
    return io.haveAlgae();
  }

  public void setHaveAlgae(boolean haveAlgae) {
    io.setHaveAlgae(haveAlgae);
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
