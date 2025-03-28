package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.CageTarget;
import frc.robot.util.Elastic;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  @AutoLogOutput(key = "Alignment/Cage Target")
  @Getter
  @Setter
  private CageTarget selectedCageTarget = CageTarget.MID;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void deployClimber() {
    Elastic.selectTab("Climb");
    io.deployClimber();
  }

  public void stopClimber() {
    io.stopClimber();
  }

  public void climb() {
    io.climb();
  }

  public void setRelayState(Relay.Value newState) {
    io.setRelayState(newState);
  }

  public boolean isClimberDeployed() {
    return io.isClimberDeployed();
  }

  public boolean isClimbingFinished() {
    return io.isClimbingFinished();
  }
}
