package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.AlignmentUtils.ReefFaceSelection.PolePosition;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class that detects when a scoring event happens (i.e., the robot was carrying coral and
 * then no longer is) and logs: 1. The face ID of the reef 2. Whether it's A-LEFT or B-RIGHT pole on
 * that face 3. Which elevator level (using the CoralSystemPresets) 4. The match time
 */
public class ReefScoringLogger {

  // A simple struct-like class to hold one scoring event
  public static class ScoringEvent {
    public final int faceId;
    public final PolePosition pole;
    public final CoralSystemPresets preset;
    public final double matchTime;

    public ScoringEvent(
        int faceId, PolePosition pole, CoralSystemPresets preset, double matchTime) {
      this.faceId = faceId;
      this.pole = pole;
      this.preset = preset;
      this.matchTime = matchTime;
    }

    @Override
    public String toString() {
      return "ScoringEvent { "
          + "FaceID="
          + faceId
          + ", Pole="
          + pole
          + ", Preset="
          + preset.name()
          + ", MatchTime="
          + matchTime
          + " }";
    }
  }

  // Keep a static list of events if you want a global logger
  private static final List<ScoringEvent> events = new ArrayList<>();

  // Track the previous value so we detect transitions
  private static boolean prevCoralInRobot = false;

  /**
   * Call this periodically (or whenever you think a score might happen). The method will check if
   * coral was in the robot and now is gone, indicating a scoring attempt. If so, it deduces
   * face/pole/level/time and logs it.
   */
  public static void checkAndLogScoringEvent(Pose2d currentPose, CoralSystem coralSystem) {
    boolean currentCoralInRobot = coralSystem.isCoralInRobot();

    // If we used to have coral, and now we don't, that's a potential scoring event
    if (prevCoralInRobot && !currentCoralInRobot) {
      // 1) Find the closest reef face
      AlignmentUtils.ReefFaceSelection faceSelection =
          AlignmentUtils.findClosestReefFaceAndRejectOthers(currentPose);
      Integer faceId = faceSelection.getAcceptedFaceId();
      if (faceId == null) {
        // Couldn’t find a valid face for some reason
        return;
      }

      // 2) Determine A-LEFT or B-RIGHT pole
      PolePosition polePosition = AlignmentUtils.findPolePosition(faceId, currentPose);

      // 3) Figure out the elevator preset from the coral system
      CoralSystemPresets currentPreset = coralSystem.getCurrentCoralPreset();

      // 4) Get the match time
      double matchTime = DriverStation.getMatchTime();

      // 5) Create a ScoringEvent and store it
      ScoringEvent event = new ScoringEvent(faceId, polePosition, currentPreset, matchTime);
      events.add(event);

      // 6) Log it to AdvantageKit
      Logger.recordOutput("ReefScoringEvent/FaceID", faceId);
      Logger.recordOutput(
          "ReefScoringEvent/Pole", polePosition.name()); // Log as "A_LEFT" or "B_RIGHT"
      Logger.recordOutput("ReefScoringEvent/ElevatorPreset", currentPreset.name());
      Logger.recordOutput("ReefScoringEvent/MatchTime", matchTime);

      // 7) Log the entire event in a structured format
      Logger.recordOutput("ReefScoringEvent/WholeEvent", event.toString()); // Logs as a string
    }

    // Update the previous state
    prevCoralInRobot = currentCoralInRobot;
  }

  /** If you want to retrieve the full list of scored events later on. */
  public static List<ScoringEvent> getScoringEvents() {
    return events;
  }
}
