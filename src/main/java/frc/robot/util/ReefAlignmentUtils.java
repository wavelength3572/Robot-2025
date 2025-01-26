package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.ChosenOrientation;
import frc.robot.Constants.ReefFacesBlue;
import frc.robot.Constants.ReefFacesRed;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class ReefAlignmentUtils {

  public static class ReefFaceSelection {
    private final Integer acceptedFaceId; // <-- New field for the face ID
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
      aprilTagMap = Constants.BLUE_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = Constants.RED_APRIL_TAGS;
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

    // 7. Return the result
    return new ReefFaceSelection(acceptedFaceId, acceptedFace, acceptedDistance, rejectedFaces);
  }

  public static ChosenOrientation pickClosestOrientationForFace(Pose2d robotPose, int faceId) {
    // 1) Get current alliance as an Optional
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    // 2) Pick the correct orientation map based on alliance
    Map<Integer, Rotation2d[]> orientationMap;
    if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Blue) {
      orientationMap = Constants.REEF_FACE_ORIENTATION_BLUE;
    } else if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
      orientationMap = Constants.REEF_FACE_ORIENTATION_RED;
    } else {
      // e.g. Alliance.Invalid or empty Optional
      // Decide how you want to handle this case:
      // (a) default to BLUE,
      // (b) return a "neutral" orientation,
      // (c) throw an exception, etc.
      // Here, let's just default to BLUE:
      orientationMap = Constants.REEF_FACE_ORIENTATION_BLUE;
    }

    // 3) Retrieve the possible orientations for this faceId
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = orientationMap.get(faceId);

    // 4) If no orientation data, provide a fallback
    if (possibleOrients == null || possibleOrients.length < 2) {
      // Return a "neutral" result indicating no data is available
      return new ChosenOrientation(currentHeading, Constants.ReefOrientationType.FRONT);
    }

    // 5) orientationA is the "front", orientationB is the "back"
    Rotation2d orientationA = possibleOrients[0];
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    // 6) Pick whichever orientation is closer
    if (diffA <= diffB) {
      // Closer to "front" orientation
      return new ChosenOrientation(orientationA, Constants.ReefOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new ChosenOrientation(orientationB, Constants.ReefOrientationType.BACK);
    }
  }

  /**
   * Calculates the target pose for driving to a pole based on the alliance, face ID, and pole
   * selection.
   *
   * @param drive The drive subsystem, providing the current pose and reef face data.
   * @param faceId The ID of the reef face to align with.
   * @param isLeftPole True for the left pole, false for the right pole.
   * @param alliance The current alliance of the robot.
   * @return The calculated target pose, or null if a valid pose cannot be determined.
   */
  public static Pose2d calculateTargetPose(
      Drive drive,
      int faceId,
      boolean isLeftPole,
      Optional<DriverStation.Alliance> alliance,
      ChosenOrientation chosen) {
    Translation2d poleTranslation;

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      // Use Blue reef data
      ReefFacesBlue blueFace = ReefFacesBlue.fromId(faceId);
      if (blueFace == null) {
        System.out.println("ReefAlignmentUtils: No matching Blue face for face ID: " + faceId);
        return null;
      }

      // Determine the pole translation based on the front/back and left/right
      // selection
      poleTranslation =
          (chosen.orientationType() == Constants.ReefOrientationType.FRONT)
              ? (isLeftPole
                  ? blueFace.getLeftPole().getFrontTranslation()
                  : blueFace.getRightPole().getFrontTranslation())
              : (isLeftPole
                  ? blueFace.getLeftPole().getBackTranslation()
                  : blueFace.getRightPole().getBackTranslation());
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // Use Red reef data
      ReefFacesRed redFace = ReefFacesRed.fromId(faceId);
      if (redFace == null) {
        System.out.println("ReefAlignmentUtils: No matching Red face for face ID: " + faceId);
        return null;
      }

      // Determine the pole translation based on the front/back and left/right
      // selection
      poleTranslation =
          (chosen.orientationType() == Constants.ReefOrientationType.FRONT)
              ? (isLeftPole
                  ? redFace.getLeftPole().getFrontTranslation()
                  : redFace.getRightPole().getFrontTranslation())
              : (isLeftPole
                  ? redFace.getLeftPole().getBackTranslation()
                  : redFace.getRightPole().getBackTranslation());
    } else {
      System.out.println("ReefAlignmentUtils: Invalid or unknown alliance.");
      return null;
    }

    // Return the target pose with the chosen orientation
    return new Pose2d(poleTranslation, chosen.rotation2D());
  }
}
