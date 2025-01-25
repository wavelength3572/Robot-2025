package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ChosenOrientation;
import frc.robot.Constants.ReefOrientationType;
import java.util.HashMap;
import java.util.Map;

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

  public static ReefFaceSelection findClosestReefFaceAndRejectOthers(
      Pose2d robotPose, Map<Integer, Translation2d> aprilTag) {
    // Edge case: If no faces exist
    if (aprilTag.isEmpty()) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    // 1. Compute distances: faceId -> distance
    Map<Integer, Double> distanceMap = new HashMap<>();
    for (Map.Entry<Integer, Translation2d> entry : aprilTag.entrySet()) {
      int faceId = entry.getKey();
      Translation2d faceTranslation = entry.getValue();

      double dx = faceTranslation.getX() - robotPose.getX();
      double dy = faceTranslation.getY() - robotPose.getY();
      double distance = Math.hypot(dx, dy);

      distanceMap.put(faceId, distance);
    }

    // 2. Find the minimum-distance entry (accepted)
    Map.Entry<Integer, Double> minEntry = null;
    for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()) {
      if (minEntry == null || entry.getValue() < minEntry.getValue()) {
        minEntry = entry;
      }
    }

    // Sanity check (should never be null if reefFaces is not empty)
    if (minEntry == null) {
      return new ReefFaceSelection(null, null, Double.NaN, new Translation2d[0]);
    }

    // 3. Accepted face data
    int acceptedFaceId = minEntry.getKey();
    double acceptedDistance = minEntry.getValue();
    Translation2d acceptedFace = aprilTag.get(acceptedFaceId);

    // 4. Build array of "rejected" faces
    distanceMap.remove(acceptedFaceId); // remove the accepted face from distanceMap

    // The remaining keys in distanceMap are "rejected"
    Translation2d[] rejectedFaces =
        distanceMap.keySet().stream()
            .map(aprilTag::get) // convert faceId -> Translation2d
            .toArray(Translation2d[]::new);

    // 5. Return the result (now includes acceptedFaceId)
    return new ReefFaceSelection(acceptedFaceId, acceptedFace, acceptedDistance, rejectedFaces);
  }

  public static ChosenOrientation pickClosestOrientationForFace(Pose2d robotPose, int faceId) {
    Rotation2d currentHeading = robotPose.getRotation();
    Rotation2d[] possibleOrients = Constants.REEF_FACE_ORIENTATION_BLUE.get(faceId);

    if (possibleOrients == null || possibleOrients.length < 2) {
      // No valid orientation data, fallback or error
      // Return a "neutral" result that indicates no data was available
      // Here, for example, we'll return the current heading as angle and `FRONT` by default
      return new ChosenOrientation(currentHeading, Constants.ReefOrientationType.FRONT);
    }

    // orientationA is the "front"
    Rotation2d orientationA = possibleOrients[0];
    // orientationB is the "back"
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    if (diffA <= diffB) {
      // Closer to the 'front' orientation
      return new ChosenOrientation(orientationA, ReefOrientationType.FRONT);
    } else {
      // Closer to the 'back' orientation
      return new ChosenOrientation(orientationB, ReefOrientationType.BACK);
    }
  }
}
