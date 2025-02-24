package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.coral.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class PickupCoralCommand extends Command {

  private final Intake intake;
  private final CoralSystem coralSystem;

  public PickupCoralCommand(CoralSystem coralSystem) {
    this.intake = coralSystem.getIntake();
    this.coralSystem = coralSystem;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/PickupCoralCommand", "Initializing - starting to pull coral");
    intake.pullCoral();
    Logger.recordOutput("Commands/PickupCoralCommand", "Initializing - pickup preset");
    coralSystem.setTargetPreset(CoralSystemPresets.PICKUP);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    boolean coralInRobot = intake.getCoralInRobot();
    if (coralInRobot) {
      Logger.recordOutput("Commands/PickupCoralCommand", "Coral detected, command finishing.");
    }
    return coralInRobot;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("Commands/PickupCoralCommand", "Interrupted - stopping intake.");
    } else {
      Logger.recordOutput("Commands/PickupCoralCommand", "Completed - stopping intake.");
    }
    intake.stopIntake();
  }
}
