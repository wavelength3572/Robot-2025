package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CageTarget;
import frc.robot.FieldConstants.ReefChosenOrientation;
import frc.robot.FieldConstants.ReefFacesBlue;
import frc.robot.FieldConstants.ReefFacesRed;
import frc.robot.FieldConstants.StationChosenOrientation;
import frc.robot.util.AlignmentUtils.ReefFaceSelection.PolePosition;
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

    public enum PolePosition {
      A_LEFT,
      B_RIGHT
    }

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
      aprilTagMap = FieldConstants.BLUE_REEF_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = FieldConstants.RED_REEF_APRIL_TAGS;
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
      orientationMap = FieldConstants.REEF_FACE_ORIENTATION_BLUE;
    } else if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
      orientationMap = FieldConstants.REEF_FACE_ORIENTATION_RED;
    } else {
      // e.g. Alliance.Invalid or empty Optional
      // Decide how you want to handle this case:
      // (a) default to BLUE,
      // (b) return a "neutral" orientation,
      // (c) throw an exception, etc.
      // Here, let's just default to BLUE:
      orientationMap = FieldConstants.REEF_FACE_ORIENTATION_BLUE;
    }

    // 3) Retrieve the possible orientations for this faceId
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = orientationMap.get(faceId);

    // 4) If no orientation data, provide a fallback
    if (possibleOrients == null || possibleOrients.length < 2) {
      // Return a "neutral" result indicating no data is available
      return new ReefChosenOrientation(currentHeading, FieldConstants.ReefOrientationType.FRONT);
    }

    // 5) orientationA is the "front", orientationB is the "back"
    Rotation2d orientationA = possibleOrients[0];
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    // 6) Pick whichever orientation is closer
    if (diffA <= diffB) {
      // Closer to "front" orientation
      return new ReefChosenOrientation(orientationA, FieldConstants.ReefOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new ReefChosenOrientation(orientationB, FieldConstants.ReefOrientationType.BACK);
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
      aprilTagMap = FieldConstants.BLUE_CORALSTATION_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = FieldConstants.RED_CORALSTATION_APRIL_TAGS;
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
      orientationMap = FieldConstants.CORAL_STATION_ORIENTATION_BLUE;
    } else if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
      orientationMap = FieldConstants.CORAL_STATION_ORIENTATION_RED;
    } else {
      // e.g. Alliance.Invalid or empty Optional
      // Decide how you want to handle this case:
      // (a) default to BLUE,
      // (b) return a "neutral" orientation,
      // (c) throw an exception, etc.
      // Here, let's just default to BLUE:
      orientationMap = FieldConstants.CORAL_STATION_ORIENTATION_BLUE;
    }

    // 3) Retrieve the possible orientations for this faceId
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = orientationMap.get(faceId);

    // 4) If no orientation data, provide a fallback
    if (possibleOrients == null || possibleOrients.length < 2) {
      // Return a "neutral" result indicating no data is available
      return new StationChosenOrientation(
          currentHeading, FieldConstants.StationOrientationType.FRONT);
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
          orientationA, FieldConstants.StationOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new StationChosenOrientation(orientationB, FieldConstants.StationOrientationType.BACK);
    }
  }

  /**
   * Returns a fixed cage alignment target.
   *
   * <p>This method creates a CageSelection that represents the fixed target for aligning to the
   * cage. The target rotation is locked based on the alliance: 0° for Blue and 180° for Red.
   *
   * @param robotPose The current robot Pose2d (used here only to compute the distance, if desired).
   * @param cageTranslation The known field position (Translation2d) of the cage (or cage center).
   * @return A CageSelection representing the fixed cage alignment target.
   */
  public static CageSelection getCageAlignmentTarget(
      Pose2d robotPose, Translation2d cageTranslation) {
    // Determine the target orientation based on alliance.
    double targetAngleRadians = 0.0;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      targetAngleRadians = Math.PI; // 180° for Red
    }
    Rotation2d targetRotation = new Rotation2d(targetAngleRadians);

    // Optionally, compute the distance from the robot to the cage. (This might help
    // with logging or if you need it later.)
    double distance = robotPose.getTranslation().getDistance(cageTranslation);

    // Create a CageSelection using the cage's translation, distance, and the fixed
    // target rotation.
    CageSelection target = new CageSelection(cageTranslation, distance, targetRotation);

    // Log the computed cage alignment target as a 3D pose for visualization.
    // Here we assume a fixed z-height (for example, 0.5 meters).
    double zHeight = 0.5;
    Pose3d target3d = target.getCageOpeningPose3d(zHeight);
    Logger.recordOutput("Alignment/CageAlignmentTarget/Pose3d", target3d);
    Logger.recordOutput("Alignment/CageAlignmentTarget/Distance", distance);
    Logger.recordOutput(
        "Alignment/CageAlignmentTarget/TargetAngleDeg", Math.toDegrees(targetAngleRadians));

    return target;
  }

  public static void setLeftCage() {
    FieldConstants.selectedCageTranslation = FieldConstants.getCage(CageTarget.LEFT);
  }

  public static void setMidCage() {
    FieldConstants.selectedCageTranslation = FieldConstants.getCage(CageTarget.MID);
  }

  public static void setRightCage() {
    FieldConstants.selectedCageTranslation = FieldConstants.getCage(CageTarget.RIGHT);
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

    /**
     * Computes the 2D pose of the cage opening using an offset.
     *
     * @return The Pose2d of the cage opening.
     */
    public Pose2d getCageOpeningPose2d() {
      double openingOffset = 0;
      Translation2d offset = new Translation2d(openingOffset, 0.0).rotateBy(rotationToCage);
      return new Pose2d(cageSelectionTranslation.plus(offset), rotationToCage);
    }

    /**
     * Converts the 2D opening pose into a 3D pose with a fixed z-height.
     *
     * @param zHeight The fixed z-height (in meters).
     * @return The Pose3d of the cage opening.
     */
    public Pose3d getCageOpeningPose3d(double zHeight) {
      Pose2d opening2d = getCageOpeningPose2d();
      return new Pose3d(
          new Translation3d(
              opening2d.getTranslation().getX(), opening2d.getTranslation().getY(), zHeight),
          new Rotation3d(0, 0, opening2d.getRotation().getRadians()));
    }
  }

  public static Pose2d getAlgaeRemovalTargetPose(Pose2d robotPose, ReefFaceSelection selection) {
    if (selection == null || selection.getAcceptedFaceId() == null) {
      Logger.recordOutput("Alignment/AlgaeRemoval", "No valid reef face selection found");
      return null;
    }

    int faceId = selection.getAcceptedFaceId();
    Pose2d targetPose = null;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      targetPose = FieldConstants.REEF_FACE_POSES_BLUE.get(faceId);
    } else {
      targetPose = FieldConstants.REEF_FACE_POSES_RED.get(faceId);
    }

    // Optionally refine the orientation using the current heading.
    FieldConstants.ReefChosenOrientation chosen = pickClosestOrientationForReef(robotPose, faceId);
    Pose2d refinedTargetPose = new Pose2d(targetPose.getTranslation(), chosen.rotation2D());

    Logger.recordOutput("Alignment/AlgaeRemoval", refinedTargetPose);
    return refinedTargetPose;
  }

  public static PolePosition findPolePosition(int faceId, Pose2d robotPose) {
    // Check alliance color to determine which reef data to use
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();

    Translation2d leftPoleTrans;
    Translation2d rightPoleTrans;

    if (alliance == DriverStation.Alliance.Blue) {
      ReefFacesBlue faceEnum = ReefFacesBlue.fromId(faceId);
      if (faceEnum == null) {
        return PolePosition.A_LEFT; // Default
      }
      leftPoleTrans = faceEnum.getLeftPole().getBranchTranslation();
      rightPoleTrans = faceEnum.getRightPole().getBranchTranslation();

    } else if (alliance == DriverStation.Alliance.Red) {
      ReefFacesRed faceEnum = ReefFacesRed.fromId(faceId);
      if (faceEnum == null) {
        return PolePosition.A_LEFT;
      }
      leftPoleTrans = faceEnum.getLeftPole().getBranchTranslation();
      rightPoleTrans = faceEnum.getRightPole().getBranchTranslation();

    } else {
      return PolePosition.A_LEFT; // Default if no alliance
    }

    double distLeft = robotPose.getTranslation().getDistance(leftPoleTrans);
    double distRight = robotPose.getTranslation().getDistance(rightPoleTrans);

    return (distLeft <= distRight) ? PolePosition.A_LEFT : PolePosition.B_RIGHT;
  }
}
