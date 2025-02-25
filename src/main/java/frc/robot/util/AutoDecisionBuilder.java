package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoDecisionBuilder {
  // A list of condition/command pairs
  private final List<SimpleEntry<BooleanSupplier, Command>> decisions = new ArrayList<>();
  private Command fallbackCommand = null; // command to execute if none of the conditions are met

  // Add a decision branch (condition and its corresponding command)
  public AutoDecisionBuilder addDecision(BooleanSupplier condition, Command command) {
    decisions.add(new SimpleEntry<>(condition, command));
    Logger.recordOutput("AutoDecisionBuilder", "Added decision for command: " + command);
    return this;
  }

  // Set the fallback command (e.g., just move into a safe pose)
  public AutoDecisionBuilder setFallback(Command fallback) {
    this.fallbackCommand = fallback;
    Logger.recordOutput("AutoDecisionBuilder", "Fallback command set to: " + fallback);
    return this;
  }

  // Build the nested ConditionalCommand with logging for each decision evaluation.
  public Command build() {
    if (fallbackCommand == null) {
      throw new IllegalStateException("Fallback command must be set");
    }
    Logger.recordOutput("AutoDecisionBuilder/Size", decisions.size());
    Command current = fallbackCommand;
    // Iterate in reverse so that the first added decision is evaluated first
    for (int i = decisions.size() - 1; i >= 0; i--) {
      final int decisionIndex = i; // capture index for logging inside lambda
      SimpleEntry<BooleanSupplier, Command> entry = decisions.get(i);
      BooleanSupplier originalCondition = entry.getKey();
      Command branchCommand = entry.getValue();
      Logger.recordOutput("AutoDecisionBuilder/branchCommand" + i, branchCommand.toString());

      // Wrap the condition to log its result and the current match time each time it's evaluated.
      BooleanSupplier loggingCondition =
          () -> {
            double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
            boolean conditionResult = originalCondition.getAsBoolean();
            Logger.recordOutput(
                "AutoDecisionBuilder/Decision" + decisionIndex,
                "MatchTime: " + matchTime + " | Condition result: " + conditionResult);
            return conditionResult;
          };

      // Build the conditional command, where if the condition is true, branchCommand runs; else,
      // current.
      current = new ConditionalCommand(branchCommand, current, loggingCondition);
    }
    return current;
  }
}
