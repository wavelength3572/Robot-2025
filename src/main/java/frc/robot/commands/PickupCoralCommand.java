package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PickupCoralCommand extends Command {

  private final Intake intake;

  public PickupCoralCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    // Start pulling the coral in
    // The subsystem itself will stop the motor automatically
    // once the limit switch is pressed, based on your code
    // in IntakeIOSpark.
    intake.pullCoral();
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    // As soon as the intake indicates the coral is in, we are done.
    return intake.getCoralInRobot();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the intake motor (just in case)
    // This also ensures cleanliness if the command is cancelled
    // mid-run for some reason.
    intake.stopIntake();
  }
}
