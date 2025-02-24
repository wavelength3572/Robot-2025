package frc.robot.util;

import frc.robot.subsystems.coral.CoralSystemPresets;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class CoralRPStatusLogger {

  // Define the scoring presets in the desired order: High (L4), Mid (L3), Low (L2), Shelf (L1).
  private static final CoralSystemPresets[] SCORING_LEVELS = {
    CoralSystemPresets.L4, CoralSystemPresets.L3, CoralSystemPresets.L2, CoralSystemPresets.L1
  };

  // Define custom labels for each scoring level.
  private static final Map<CoralSystemPresets, String> LEVEL_LABELS = new HashMap<>();

  static {
    LEVEL_LABELS.put(CoralSystemPresets.L4, "High");
    LEVEL_LABELS.put(CoralSystemPresets.L3, "Mid");
    LEVEL_LABELS.put(CoralSystemPresets.L2, "Low");
    LEVEL_LABELS.put(CoralSystemPresets.L1, "Shelf");
  }

  /**
   * Tallies the number of scoring events per level (L1, L2, L3, L4) and logs each level's status as
   * separate output keys for a Large Text Widget.
   *
   * @param coopertitionAchieved if true, uses a threshold of 4, otherwise 5.
   */
  public static void logCoralStatus(boolean coopertitionAchieved) {
    // Set the threshold based on whether Coopertition has been achieved.
    int threshold = coopertitionAchieved ? 4 : 5;

    // Use an EnumMap to count the number of scoring events per scoring level.
    Map<CoralSystemPresets, Integer> coralCounts = new EnumMap<>(CoralSystemPresets.class);

    // Initialize counts to 0 for each scoring level.
    for (CoralSystemPresets preset : SCORING_LEVELS) {
      coralCounts.put(preset, 0);
    }

    // Iterate over all scoring events and count them if they are scoring presets.
    for (ReefScoringLogger.ScoringEvent event : ReefScoringLogger.getScoringEvents()) {
      for (CoralSystemPresets validPreset : SCORING_LEVELS) {
        if (event.preset == validPreset) {
          coralCounts.put(validPreset, coralCounts.get(validPreset) + 1);
          break; // Once matched, move on to the next event.
        }
      }
    }

    // Log each level's count separately with its own output key.
    // Using the new order and custom labels.
    for (CoralSystemPresets preset : SCORING_LEVELS) {
      int count = coralCounts.get(preset);
      String label = LEVEL_LABELS.getOrDefault(preset, preset.name());
      // Example output: "High (L4): 3/5"
      String output = String.format("%s (%s): %d/%d", label, preset.name(), count, threshold);
      Logger.recordOutput("Scoring/CoralRPStatus/" + preset.name(), output);
    }
  }
}
