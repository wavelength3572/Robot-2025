package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.coral.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class PickupCoralCommand extends Command {

  private final double THRESHOLD_PULL_TIME_SECONDS = .5; // seconds
  private final Intake intake;
  private final CoralSystem coralSystem;
  private boolean haveTriggeredPullCoral = false;
  private Timer timer = new Timer();

  public PickupCoralCommand(CoralSystem coralSystem) {
    this.intake = coralSystem.getIntake();
    this.coralSystem = coralSystem;
  }

  @Override
  public void initialize() {
    haveTriggeredPullCoral = false;
    timer.restart();

    Logger.recordOutput(
        "Commands/PickupCoralCommand/MatchTimeStartofCommand", DriverStation.getMatchTime());
    coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
  }

  @Override
  public void execute() {
    Logger.recordOutput("Commands/PickupCoralCommand/TriggeredPull", haveTriggeredPullCoral);
    if (timer.get() > THRESHOLD_PULL_TIME_SECONDS && !haveTriggeredPullCoral) {
      haveTriggeredPullCoral = true;
      intake.pullCoral();
    }
  }

  @Override
  public boolean isFinished() {
    boolean haveCoral = intake.haveCoral();
    if (haveCoral) {
      Logger.recordOutput(
          "Commands/PickupCoralCommand/FinishedMatchTime", DriverStation.getMatchTime());
    }
    return haveCoral;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("Commands/PickupCoralCommand", "Interrupted - stopping intake.");
    } else {
      Logger.recordOutput("Commands/PickupCoralCommand", "Completed - stopping intake.");
    }
  }
}
