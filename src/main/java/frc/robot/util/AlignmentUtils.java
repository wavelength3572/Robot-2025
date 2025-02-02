package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.CommandConstants;
import frc.robot.commands.CommandConstants.CageTarget;
import frc.robot.commands.CommandConstants.ReefChosenOrientation;
import frc.robot.commands.CommandConstants.StationChosenOrientation;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

@AutoLog
public class AlignmentUtils {

  public static class ReefFaceSelection {
    private final Integer acceptedFaceId;
    private final Translation2d acceptedFace;
    private final double acceptedDistance;
    private final Translation2d[] rejectedFaces;

    public ReefFaceSelection(
        Integer acceptedFaceId,
        Translation2d acceptedFace,
        double acceptedDistance,
        Translation2d[] rejectedFaces) {
      this.acceptedFaceId = acceptedFaceId;
      this.acceptedFace = acceptedFace;
      this.acceptedDistance = acceptedDistance;
      this.rejectedFaces = rejectedFaces;
    }

    public Integer getAcceptedFaceId() {
      return acceptedFaceId;
    }

    public Translation2d getAcceptedFace() {
      return acceptedFace;
    }

    public double getAcceptedDistance() {
      return acceptedDistance;
    }

    public Translation2d[] getRejectedFaces() {
      return rejectedFaces;
    }
  }

  public static ReefFaceSelection findClosestReefFaceAndRejectOthers(Pose2d robotPose) {
    // 1. Pick the correct AprilTag map based on alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Map<Integer, Translation2d> aprilTagMap;
    if (alliance.get() == Alliance.Blue) {
      aprilTagMap = CommandConstants.BLUE_REEF_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = CommandConstants.RED_REEF_APRIL_TAGS;
    } else {
      // e.g. Alliance.Invalid or something else
      // Return a neutral "none found" result or handle as an error
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    // 2. Edge case: If no faces exist for that alliance
    if (aprilTagMap.isEmpty()) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    // 3. Compute distances: faceId -> distance from robotPose
    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTagMap.entrySet()) {
      int faceId = entry.getKey();
      Translation2d faceTranslation = entry.getValue();

      double dx = faceTranslation.getX() - robotPose.getX();
      double dy = faceTranslation.getY() - robotPose.getY();
      double distance = Math.hypot(dx, dy);

      distanceMap.put(faceId, distance);
    }

    // 4. Find the minimum-distance entry (accepted face)
    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    // 5. Sanity check
    if (minEntry == null) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    int acceptedFaceId = minEntry.getKey();
    double acceptedDistance = minEntry.getValue();
    Translation2d acceptedFace = aprilTagMap.get(acceptedFaceId);

    // 6. Build array of "rejected" faces
    distanceMap.remove(acceptedFaceId); // remove accepted face
    Translation2d[] rejectedFaces =
        distanceMap.keySet().stream().map(aprilTagMap::get).toArray(Translation2d[]::new);

    Logger.recordOutput("Alignment/Reef/ClosestDistance", acceptedDistance);
    Logger.recordOutput("Alignment/Reef/AcceptedFace", acceptedFace);
    Logger.recordOutput("Alignment/Reef/RejectedFaces", rejectedFaces);
    Logger.recordOutput("Alignment/Reef/AcceptedFaceID", acceptedFaceId);

    // 7. Return the result
    return new ReefFaceSelection(acceptedFaceId, acceptedFace, acceptedDistance, rejectedFaces);
  }

  public static ReefChosenOrientation pickClosestOrientationForReef(Pose2d robotPose, int faceId) {
    // 1) Get current alliance as an Optional
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    // 2) Pick the correct orientation map based on alliance
    Map<Integer, Rotation2d[]> orientationMap;
    if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Blue) {
      orientationMap = CommandConstants.REEF_FACE_ORIENTATION_BLUE;
    } else if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
      orientationMap = CommandConstants.REEF_FACE_ORIENTATION_RED;
    } else {
      // e.g. Alliance.Invalid or empty Optional
      // Decide how you want to handle this case:
      // (a) default to BLUE,
      // (b) return a "neutral" orientation,
      // (c) throw an exception, etc.
      // Here, let's just default to BLUE:
      orientationMap = CommandConstants.REEF_FACE_ORIENTATION_BLUE;
    }

    // 3) Retrieve the possible orientations for this faceId
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = orientationMap.get(faceId);

    // 4) If no orientation data, provide a fallback
    if (possibleOrients == null || possibleOrients.length < 2) {
      // Return a "neutral" result indicating no data is available
      return new ReefChosenOrientation(currentHeading, CommandConstants.ReefOrientationType.FRONT);
    }

    // 5) orientationA is the "front", orientationB is the "back"
    Rotation2d orientationA = possibleOrients[0];
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    // 6) Pick whichever orientation is closer
    if (diffA <= diffB) {
      // Closer to "front" orientation
      return new ReefChosenOrientation(orientationA, CommandConstants.ReefOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new ReefChosenOrientation(orientationB, CommandConstants.ReefOrientationType.BACK);
    }
  }

  public static class CoralStationSelection {
    private final Integer acceptedStationId; // April Tag ID
    private final Translation2d acceptedStation; // Translation of April corresponding to Station
    private final double acceptedDistance; // Shortest Distance to a station
    private final Translation2d[] rejectedStation; // Translation of non selected station

    public CoralStationSelection(
        Integer acceptedStationId,
        Translation2d acceptedStation,
        double acceptedDistance,
        Translation2d[] rejectedStation) {
      this.acceptedStationId = acceptedStationId;
      this.acceptedStation = acceptedStation;
      this.acceptedDistance = acceptedDistance;
      this.rejectedStation = rejectedStation;
    }

    public Integer getAcceptedStationId() {
      return acceptedStationId;
    }

    public Translation2d getAcceptedStation() {
      return acceptedStation;
    }

    public double getAcceptedDistance() {
      return acceptedDistance;
    }

    public Translation2d[] getRejectedStation() {
      return rejectedStation;
    }
  }

  public static CoralStationSelection findClosestCoralStation(Pose2d robotPose) {
    // 1. Pick the correct AprilTag map based on alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Map<Integer, Translation2d> aprilTagMap;
    if (alliance.get() == Alliance.Blue) {
      aprilTagMap = CommandConstants.BLUE_CORALSTATION_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = CommandConstants.RED_CORALSTATION_APRIL_TAGS;
    } else {
      // e.g. Alliance.Invalid or something else
      // Return a neutral "none found" result or handle as an error
      return new CoralStationSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    // 2. Compute distances: faceId -> distance from robotPose
    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTagMap.entrySet()) {
      int faceId = entry.getKey();
      Translation2d faceTranslation = entry.getValue();

      double dx = faceTranslation.getX() - robotPose.getX();
      double dy = faceTranslation.getY() - robotPose.getY();
      double distance = Math.hypot(dx, dy);

      distanceMap.put(faceId, distance);
    }

    // 3. Find the minimum-distance entry (accepted face)
    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    int acceptedStationId = minEntry.getKey();
    double acceptedDistance = minEntry.getValue();
    Translation2d acceptedStation = aprilTagMap.get(acceptedStationId);

    // 4. Build array of "rejected" faces
    distanceMap.remove(acceptedStationId); // remove accepted face
    Translation2d[] rejectedStation =
        distanceMap.keySet().stream().map(aprilTagMap::get).toArray(Translation2d[]::new);

    Logger.recordOutput("Alignment/CoralStation/ClosestDistance", acceptedDistance);
    Logger.recordOutput("Alignment/CoralStation/AcceptedStation", acceptedStation);
    Logger.recordOutput("Alignment/CoralStation/RejectedStation", rejectedStation);
    Logger.recordOutput("Alignment/CoralStation/AcceptedStationID", acceptedStation);
    // 7. Return the result
    return new CoralStationSelection(
        acceptedStationId, acceptedStation, acceptedDistance, rejectedStation);
  }

  public static StationChosenOrientation pickClosestOrientationForStation(
      Pose2d robotPose, int faceId) {
    // 1) Get current alliance as an Optional
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    // 2) Pick the correct orientation map based on alliance
    Map<Integer, Rotation2d[]> orientationMap;
    if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Blue) {
      orientationMap = CommandConstants.CORAL_STATION_ORIENTATION_BLUE;
    } else if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
      orientationMap = CommandConstants.CORAL_STATION_ORIENTATION_RED;
    } else {
      // e.g. Alliance.Invalid or empty Optional
      // Decide how you want to handle this case:
      // (a) default to BLUE,
      // (b) return a "neutral" orientation,
      // (c) throw an exception, etc.
      // Here, let's just default to BLUE:
      orientationMap = CommandConstants.CORAL_STATION_ORIENTATION_BLUE;
    }

    // 3) Retrieve the possible orientations for this faceId
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = orientationMap.get(faceId);

    // 4) If no orientation data, provide a fallback
    if (possibleOrients == null || possibleOrients.length < 2) {
      // Return a "neutral" result indicating no data is available
      return new StationChosenOrientation(
          currentHeading, CommandConstants.StationOrientationType.FRONT);
    }

    // 5) orientationA is the "front", orientationB is the "back"
    Rotation2d orientationA = possibleOrients[0];
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    // 6) Pick whichever orientation is closer
    if (diffA <= diffB) {
      // Closer to "front" orientation
      return new StationChosenOrientation(
          orientationA, CommandConstants.StationOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new StationChosenOrientation(
          orientationB, CommandConstants.StationOrientationType.BACK);
    }
  }

  /**
   * Creates a rotation supplier that computes the field-relative angle from the robot's current
   * position to the specified cage target.
   *
   * <p>This helper method assumes the robot's pose is provided in WPILib coordinates (with the blue
   * side origin at the bottom right) and returns the angle (as a Rotation2d) that points toward the
   * cage target.
   *
   * @param poseSupplier A supplier for the current robot Pose2d.
   * @param cageTarget The target cage position as a Translation2d.
   * @return A Supplier that returns the desired Rotation2d on each call.
   */
  public static CageSelection findCageRobotAngle(Pose2d robotPose, Translation2d cageTranslation) {

    // Calculate the direction vector from the robot to the cage target
    Translation2d direction = cageTranslation.minus(robotPose.getTranslation());

    // Compute the angle using atan2 (which handles WPILib's coordinate system correctly)
    double angleRadians = Math.atan2(direction.getY(), direction.getX());

    double distanceToCage = robotPose.getTranslation().minus(cageTranslation).getNorm();

    Logger.recordOutput("Alignment/Cage/CageTranslation", cageTranslation);
    Logger.recordOutput("Alignment/Cage/CageDistance", distanceToCage);
    Logger.recordOutput("Alignment/Cage/angleToCage", Units.radiansToDegrees(angleRadians));
    return new CageSelection(cageTranslation, distanceToCage, new Rotation2d(angleRadians));
  }
  ;

  public static void setLeftCage() {
    CommandConstants.selectedCageTranslation = CommandConstants.getCage(CageTarget.LEFT);
  }

  public static void setMidCage() {
    CommandConstants.selectedCageTranslation = CommandConstants.getCage(CageTarget.MID);
  }

  public static void setRightCage() {
    CommandConstants.selectedCageTranslation = CommandConstants.getCage(CageTarget.RIGHT);
  }

  public static class CageSelection {

    private final Translation2d cageSelectionTranslation;
    private final double distanceToCage;
    private final Rotation2d rotationToCage;

    public CageSelection(
        Translation2d cageSelectionTranslation, double distanceToCage, Rotation2d rotationToCage) {
      this.cageSelectionTranslation = cageSelectionTranslation;
      this.distanceToCage = distanceToCage;
      this.rotationToCage = rotationToCage;
    }

    public Translation2d getCageSelectionTranslation() {
      return cageSelectionTranslation;
    }

    public double getDistanceToCage() {
      return distanceToCage;
    }

    public Rotation2d getRotationToCage() {
      return rotationToCage;
    }
  }
}
