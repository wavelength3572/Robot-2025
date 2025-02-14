package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void pushAlgae() {
    io.pushAlgae();
  }

  public void pullAlgae() {
    io.pullAlgae();
  }

  public void stop() {
    io.stop();
  }

  public void deployAlgae() {
    io.deployAlgae();
  }

  public void stowAlgae() {
    io.stowAlgae();
  }

  public boolean getAlgaeInRobot() {
    return io.getAlgaeInRobot();
  }

  public void setAlgaeInRobot(boolean algaeInRobot) {
    io.setAlgaeInRobot(algaeInRobot);
  }

  public double getCurrentAngleDEG() {
    return io.getCurrentAngleDEG();
  }

  public double getCurrentSpeedRPM() {
    return io.getCurrentSpeedRPM();
  }

  /** Sets the target deploy arm angle in degrees */
  public void setTargetAngleDEG(double angle) {
    io.setTargetAngleDEG(angle);
  }

  /** Sets the intake/outtake motor speed */
  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
}
