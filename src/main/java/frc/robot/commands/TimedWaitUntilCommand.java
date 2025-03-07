package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** A command that waits until a specified condition is true and logs the elapsed time. */
public class TimedWaitUntilCommand extends Command {
  private final BooleanSupplier condition;
  private final String conditionalName;
  private double startTime;
  // Execution counter that increments on each execution.
  private int executionInstanceId;
  private static int executionCounter = 0;

  /**
   * Constructs a new TimedWaitUntilCommand.
   *
   * @param conditionalName A name to help differentiate this command in the logs.
   * @param condition A BooleanSupplier that returns true when the command should finish.
   */
  public TimedWaitUntilCommand(String conditionalName, BooleanSupplier condition) {
    this.conditionalName = conditionalName;
    this.condition = condition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Record the start time (in seconds).
    startTime = Timer.getFPGATimestamp();
    // Update the execution counter for each run.
    executionInstanceId = executionCounter++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return condition.getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    // Log the elapsed time in milliseconds under a key that includes the conditional name and
    // unique execution instance ID.
    Logger.recordOutput(
        "Commands/TimedWaitUntilCommand/"
            + conditionalName
            + "/Execution"
            + executionInstanceId
            + "/ElapsedTime(s)",
        elapsedTime);
  }
}
