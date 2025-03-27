package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

public class ChooserAlignmentCommand extends Command {
  private final SendableChooser<Function<Boolean, Command>> chooser;
  private final boolean isLeftPole;
  private Command delegatedCommand;

  public ChooserAlignmentCommand(
      SendableChooser<Function<Boolean, Command>> chooser, boolean isLeftPole) {
    this.chooser = chooser;
    this.isLeftPole = isLeftPole;
    // Optionally add requirements if needed:
    // addRequirements(drive, coralSystem);
  }

  @Override
  public void initialize() {
    // Capture the selected command once when starting.
    delegatedCommand = chooser.getSelected().apply(isLeftPole);
    if (delegatedCommand != null) {
      delegatedCommand.initialize();
    }
  }

  @Override
  public void execute() {
    if (delegatedCommand != null) {
      delegatedCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return delegatedCommand == null || delegatedCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (delegatedCommand != null) {
      delegatedCommand.end(interrupted);
    }
  }
}
