package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void deployClimber() {
    io.deployClimber();
  }

  public void stopClimber() {

  }

  public void stowClimber() {
    io.stowClimber();
  }

  public boolean isClimberDeployed() {
    return io.isClimberDeployed();
  }
}
