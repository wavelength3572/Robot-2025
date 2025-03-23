package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
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

// 3-20 and 3-21 changes:
// Auto Scoring
// 1) if driver auto aligns - checks if we are inConfiguration, haveSeenTag recently, and aligned
// 2) if operator goes to score config - scores if haveSeenReefTag recently and aligned
// L1 auto scoring only on driver button (easy to remove)
// smart drive working for L1
// cagealignment - no buttons for alignment (just cares about climber and dist to mid cage)
// Arm goes safe position when capturing algae
// algae processing orientation for smart drive

// pulls in coral for 1 second if it thinks it has coral already - this is tricky because of how
// intake is coded
// add TOF on front for coral on reef and correction?
// investigate algae dislodge (do we need to remove driver control?)
// should we try something else?

@AutoLog
public class AlignmentUtils {

  public static Transform2d ALGAE_OFFSET =
      new Transform2d(new Translation2d(.025, 0), new Rotation2d(0));

  public static class ReefFaceSelection {
    private final Integer acceptedFaceId;
    private final Translation2d acceptedFace;
    private final double acceptedDistance;
    private final Translation2d[] rejectedFaces;
    private final boolean tagSeenRecently;

    public enum PolePosition {
      A_LEFT,
      B_RIGHT
    }

    public ReefFaceSelection(
        Integer acceptedFaceId,
        Translation2d acceptedFace,
        double acceptedDistance,
        Translation2d[] rejectedFaces,
        boolean tagSeenRecently) {
      this.acceptedFaceId = acceptedFaceId;
      this.acceptedFace = acceptedFace;
      this.acceptedDistance = acceptedDistance;
      this.rejectedFaces = rejectedFaces;
      this.tagSeenRecently = tagSeenRecently;
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

    public boolean getTagSeenRecently() {
      return tagSeenRecently;
    }
  }

  public static ReefFaceSelection findClosestReefFaceAndRejectOthers(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Map<Integer, Translation2d> aprilTagMap;
    Map<Integer, Rotation2d[]> faceOrientationMap;

    if (alliance.get() == Alliance.Blue) {
      aprilTagMap = FieldConstants.BLUE_REEF_APRIL_TAGS;
      faceOrientationMap = FieldConstants.REEF_FACE_ORIENTATION_BLUE;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = FieldConstants.RED_REEF_APRIL_TAGS;
      faceOrientationMap = FieldConstants.REEF_FACE_ORIENTATION_RED;
    } else {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0], false);
    }

    if (aprilTagMap.isEmpty()) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0], false);
    }

    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTagMap.entrySet()) {
      int faceId = entry.getKey();
      Translation2d faceTranslation = entry.getValue();
      double distance = robotPose.getTranslation().getDistance(faceTranslation);
      distanceMap.put(faceId, distance);
    }

    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    if (minEntry == null) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0], false);
    }

    int acceptedFaceId = minEntry.getKey();
    Translation2d acceptedFace = aprilTagMap.get(acceptedFaceId);

    // ✅ Check if the Reef Face AprilTag has been seen within the last 8 seconds
    boolean tagSeenRecently = RobotStatus.hasRecentlySeenAprilTag(acceptedFaceId, 3.0);
    Logger.recordOutput("Alignment/Reef/TagSeenRecently", tagSeenRecently);

    // Get the Reef face orientation based on alliance color
    Rotation2d faceOrientation = faceOrientationMap.get(acceptedFaceId)[0];

    // Compute perpendicular distance using shared helper
    double perpendicularDistance =
        getPerpendicularDistance(robotPose.getTranslation(), acceptedFace, faceOrientation);

    distanceMap.remove(acceptedFaceId);
    Translation2d[] rejectedFaces =
        distanceMap.keySet().stream().map(aprilTagMap::get).toArray(Translation2d[]::new);

    Logger.recordOutput("Alignment/Reef/ClosestDistance", minEntry.getValue());
    Logger.recordOutput("Alignment/Reef/PerpendicularDistance", perpendicularDistance);
    Logger.recordOutput("Alignment/Reef/AcceptedFace", acceptedFace);
    Logger.recordOutput("Alignment/Reef/RejectedFaces", rejectedFaces);
    Logger.recordOutput("Alignment/Reef/AcceptedFaceID", acceptedFaceId);

    return new ReefFaceSelection(
        acceptedFaceId, acceptedFace, perpendicularDistance, rejectedFaces, tagSeenRecently);
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
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Map<Integer, Translation2d> aprilTagMap;
    Map<Integer, Rotation2d[]> stationOrientationMap;

    if (alliance.get() == Alliance.Blue) {
      aprilTagMap = FieldConstants.BLUE_CORALSTATION_APRIL_TAGS;
      stationOrientationMap = FieldConstants.CORAL_STATION_ORIENTATION_BLUE;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = FieldConstants.RED_CORALSTATION_APRIL_TAGS;
      stationOrientationMap = FieldConstants.CORAL_STATION_ORIENTATION_RED;
    } else {
      return new CoralStationSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    if (aprilTagMap.isEmpty()) {
      return new CoralStationSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTagMap.entrySet()) {
      int stationId = entry.getKey();
      Translation2d stationTranslation = entry.getValue();
      double distance = robotPose.getTranslation().getDistance(stationTranslation);
      distanceMap.put(stationId, distance);
    }

    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    if (minEntry == null) {
      return new CoralStationSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    int acceptedStationId = minEntry.getKey();
    Translation2d acceptedStation = aprilTagMap.get(acceptedStationId);

    // Get the Coral Station face orientation based on alliance color
    Rotation2d stationOrientation = stationOrientationMap.get(acceptedStationId)[0];

    // Compute perpendicular distance using shared helper
    double perpendicularDistance =
        getPerpendicularDistance(robotPose.getTranslation(), acceptedStation, stationOrientation);

    distanceMap.remove(acceptedStationId);
    Translation2d[] rejectedStations =
        distanceMap.keySet().stream().map(aprilTagMap::get).toArray(Translation2d[]::new);

    Logger.recordOutput("Alignment/CoralStation/ClosestDistance", minEntry.getValue());
    Logger.recordOutput("Alignment/CoralStation/PerpendicularDistance", perpendicularDistance);
    Logger.recordOutput("Alignment/CoralStation/AcceptedStation", acceptedStation);
    Logger.recordOutput("Alignment/CoralStation/RejectedStations", rejectedStations);
    Logger.recordOutput("Alignment/CoralStation/AcceptedStationID", acceptedStationId);

    return new CoralStationSelection(
        acceptedStationId, acceptedStation, perpendicularDistance, rejectedStations);
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
    Logger.recordOutput("Alignment/TargetAlgaePose", targetPose);

    // Optionally refine the orientation using the current heading.
    FieldConstants.ReefChosenOrientation chosen = pickClosestOrientationForReef(robotPose, faceId);
    Pose2d reefFacePose = new Pose2d(targetPose.getTranslation(), chosen.rotation2D());

    Pose2d algaeRemovalPose = reefFacePose.plus(ALGAE_OFFSET);

    Logger.recordOutput("Alignment/AlgaeRemoval", algaeRemovalPose);
    return algaeRemovalPose;
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

  public static double getPerpendicularDistance(
      Translation2d robotPosition,
      Translation2d referencePosition,
      Rotation2d referenceOrientation) {
    // Convert Rotation2d to a unit normal vector (direction the face is pointing)
    Translation2d normalVector =
        new Translation2d(
            Math.cos(referenceOrientation.getRadians()),
            Math.sin(referenceOrientation.getRadians()));

    // Compute the vector from the reference position (reef/station) to the robot
    Translation2d referenceToRobotVector = robotPosition.minus(referencePosition);

    // Project the reference-to-robot vector onto the normal vector
    return Math.abs(
        referenceToRobotVector.getX() * normalVector.getX()
            + referenceToRobotVector.getY() * normalVector.getY());
  }

  public static class ProcessorSelection {
    private final Translation2d processorSelectionTranslation;
    private final double distanceToProcessor;
    private final Rotation2d rotationToProcessor;

    public ProcessorSelection(
        Translation2d processorSelectionTranslation,
        double distanceToProcessor,
        Rotation2d rotationToProcessor) {
      this.processorSelectionTranslation = processorSelectionTranslation;
      this.distanceToProcessor = distanceToProcessor;
      this.rotationToProcessor = rotationToProcessor;
    }

    public Translation2d getProcessorSelectionTranslation() {
      return processorSelectionTranslation;
    }

    public double getDistanceToProcessor() {
      return distanceToProcessor;
    }

    public Rotation2d getRotationToProcessor() {
      return rotationToProcessor;
    }
  }

  public static ProcessorSelection findClosestProcessor(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Map<Integer, Translation2d> aprilTagMap;
    Rotation2d processorOrientation;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      aprilTagMap = FieldConstants.BLUE_PROCESSOR_APRIL_TAG;
      processorOrientation = FieldConstants.PROCESSOR_ORIENTATION_BLUE;
    } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      aprilTagMap = FieldConstants.RED_PROCESSOR_APRIL_TAG;
      processorOrientation = FieldConstants.PROCESSOR_ORIENTATION_RED;
    } else {
      return new ProcessorSelection(null, Double.NaN, null);
    }

    if (aprilTagMap.isEmpty()) {
      return new ProcessorSelection(null, Double.NaN, null);
    }

    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTagMap.entrySet()) {
      int processorId = entry.getKey();
      Translation2d processorTranslation = entry.getValue();
      double distance = robotPose.getTranslation().getDistance(processorTranslation);
      distanceMap.put(processorId, distance);
    }

    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    if (minEntry == null) {
      return new ProcessorSelection(null, Double.NaN, null);
    }

    int acceptedProcessorId = minEntry.getKey();
    Translation2d acceptedProcessor = aprilTagMap.get(acceptedProcessorId);

    Logger.recordOutput("Alignment/Processor/ClosestDistance", minEntry.getValue());
    Logger.recordOutput("Alignment/Processor/AcceptedProcessor", acceptedProcessor);
    Logger.recordOutput("Alignment/Processor/AcceptedProcessorID", acceptedProcessorId);
    Logger.recordOutput("Alignment/Processor/ProcessorOrientation", processorOrientation);

    return new ProcessorSelection(acceptedProcessor, minEntry.getValue(), processorOrientation);
  }
}
