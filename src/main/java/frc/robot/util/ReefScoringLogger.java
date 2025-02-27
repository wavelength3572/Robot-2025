package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
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

  public static class ScoringEvent {
    public final int faceId;
    public final PolePosition pole;
    public final CoralSystemPresets preset;
    public final double matchTime;
    public final Pose2d robotPose; // new field

    public ScoringEvent(
        int faceId,
        PolePosition pole,
        CoralSystemPresets preset,
        double matchTime,
        Pose2d robotPose) {
      this.faceId = faceId;
      this.pole = pole;
      this.preset = preset;
      this.matchTime = matchTime;
      this.robotPose = robotPose;
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
          + ", RobotPose="
          + robotPose
          + " }";
    }
  }

  // Keep a static list of events if you want a global logger
  private static final List<ScoringEvent> events = new ArrayList<>();

  // Track the previous value of coral in robot so we detect transitions
  private static boolean prevHaveCoral = false;

  public static void checkAndLogScoringEvent(Pose2d currentPose, CoralSystem coralSystem) {
    boolean currentHaveCoral = coralSystem.isHaveCoral();

    // If we used to have coral, and now we don't, that's a potential scoring event
    if (prevHaveCoral && !currentHaveCoral) {
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
      ScoringEvent event =
          new ScoringEvent(faceId, polePosition, currentPreset, matchTime, currentPose);
      events.add(event);

      // 6) Mark the corresponding coral as scored in our coralMapping.
      FieldConstants.CoralKey key =
          new FieldConstants.CoralKey(faceId, polePosition, currentPreset);
      if (FieldConstants.coralMapping.containsKey(key)) {
        FieldConstants.coralMapping.get(key).scored = true;
      } else {
        Logger.recordOutput(
            "Scoring/ReefScoringLogger/Warning", "No coral mapping found for key: " + key);
      }

      // 7) Log it to AdvantageKit
      Logger.recordOutput("Scoring/ReefScoringEvent/FaceID", faceId);
      Logger.recordOutput(
          "Scoring/ReefScoringEvent/Pole", polePosition.name()); // Log as "A_LEFT" or "B_RIGHT"
      Logger.recordOutput("Scoring/ReefScoringEvent/ElevatorPreset", currentPreset.name());
      Logger.recordOutput("Scoring/ReefScoringEvent/MatchTime", matchTime);
      Logger.recordOutput("Scoring/ReefScoringEvent/RobotScoringPose", currentPose);

      // 8) Log the entire event in a structured format
      Logger.recordOutput(
          "Scoring/ReefScoringEvent/WholeEvent", event.toString()); // Logs as a string
    }

    // Update the previous state
    prevHaveCoral = currentHaveCoral;
  }

  /** If you want to retrieve the full list of scored events later on. */
  public static List<ScoringEvent> getScoringEvents() {
    return events;
  }

  public static void clearScoringEvents() {
    events.clear();
  }
}
