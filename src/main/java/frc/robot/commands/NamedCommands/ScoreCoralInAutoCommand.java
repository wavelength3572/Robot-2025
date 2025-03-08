package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class ScoreCoralInAutoCommand extends Command {
  private static final double MAX_OUTTAKE_TIME = 0.6; // total hard stop, in seconds
  private static final double ADDITIONAL_CLEAR_TIME =
      0.15; // how long to keep running after the coral is gone

  private final Intake intake;
  private final Timer timer = new Timer();

  private boolean coralCleared = false;
  private double timeWhenCoralCleared = 0.0;

  public ScoreCoralInAutoCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    // Start pushing the coral out
    intake.pushCoral();

    // Reset and start the timer
    timer.reset();
    timer.start();

    coralCleared = false;
    timeWhenCoralCleared = 0.0;
  }

  @Override
  public void execute() {
    // Log the current time and sensor state
    Logger.recordOutput("ScoreCoral/CurrentTime", timer.get());
    Logger.recordOutput("ScoreCoral/HaveCoral", intake.haveCoral());
    Logger.recordOutput("ScoreCoral/CoralCleared", coralCleared);

    // 1) Check limit switch (or sensor) to see if the piece has left the robot
    if (!intake.haveCoral() && !coralCleared) {
      // The moment we see it's gone, record the time
      coralCleared = true;
      timeWhenCoralCleared = timer.get();
      Logger.recordOutput("ScoreCoral/TimeWhenCoralCleared", timeWhenCoralCleared);
    }
  }

  @Override
  public boolean isFinished() {
    // 2) If we reached the total max time, we stop no matter what
    if (timer.get() >= MAX_OUTTAKE_TIME) {
      Logger.recordOutput("ScoreCoral/FinishReason", "MaxOuttakeTimeExceeded");

      return true;
    }

    // 3) If we already saw the coral clear, check if we have run
    //    for an additional X seconds *after* clearing
    if (coralCleared) {
      double timeSinceCleared = timer.get() - timeWhenCoralCleared;
      if (timeSinceCleared >= ADDITIONAL_CLEAR_TIME) {
        // Log the reason for finishing
        Logger.recordOutput("ScoreCoral/finalTime", timeSinceCleared);
        return true;
      }
    }

    // Otherwise, we haven't detected it leaving yet, keep running
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the outtake
    timer.stop();
  }

  public static double getExpectedDuration() {
    return ADDITIONAL_CLEAR_TIME;
  }
}
