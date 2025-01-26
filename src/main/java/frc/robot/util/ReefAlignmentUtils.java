package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.CommandConstants;
import frc.robot.commands.CommandConstants.ChosenOrientation;
import frc.robot.commands.CommandConstants.ReefFacesBlue;
import frc.robot.commands.CommandConstants.ReefFacesRed;
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
      aprilTagMap = CommandConstants.BLUE_APRIL_TAGS;
    } else if (alliance.get() == Alliance.Red) {
      aprilTagMap = CommandConstants.RED_APRIL_TAGS;
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
      return new ChosenOrientation(currentHeading, CommandConstants.ReefOrientationType.FRONT);
    }

    // 5) orientationA is the "front", orientationB is the "back"
    Rotation2d orientationA = possibleOrients[0];
    Rotation2d orientationB = possibleOrients[1];

    double diffA = Math.abs(currentHeading.minus(orientationA).getRadians());
    double diffB = Math.abs(currentHeading.minus(orientationB).getRadians());

    // 6) Pick whichever orientation is closer
    if (diffA <= diffB) {
      // Closer to "front" orientation
      return new ChosenOrientation(orientationA, CommandConstants.ReefOrientationType.FRONT);
    } else {
      // Closer to "back" orientation
      return new ChosenOrientation(orientationB, CommandConstants.ReefOrientationType.BACK);
    }
  }

  public static Pose2d calculateTargetPose(
      Drive drive,
      int faceId,
      boolean isLeftPole,
      Optional<DriverStation.Alliance> alliance,
      ChosenOrientation chosen,
      double offsetMeters) { // Added offset parameter
    Translation2d poleTranslation;

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      ReefFacesBlue blueFace = ReefFacesBlue.fromId(faceId);
      if (blueFace == null) {
        System.out.println("Invalid face ID for Blue alliance.");
        return null;
      }
      // Select left/right and front/back
      poleTranslation =
          isLeftPole
              ? chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? blueFace.getLeftPole().getFrontTranslation()
                  : blueFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? blueFace.getRightPole().getFrontTranslation()
                  : blueFace.getRightPole().getBackTranslation();

    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      ReefFacesRed redFace = ReefFacesRed.fromId(faceId);
      if (redFace == null) {
        System.out.println("Invalid face ID for Red alliance.");
        return null;
      }
      // Select left/right and front/back
      poleTranslation =
          isLeftPole
              ? chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? redFace.getLeftPole().getFrontTranslation()
                  : redFace.getLeftPole().getBackTranslation()
              : chosen.orientationType() == CommandConstants.ReefOrientationType.FRONT
                  ? redFace.getRightPole().getFrontTranslation()
                  : redFace.getRightPole().getBackTranslation();
    } else {
      System.out.println("Unknown alliance. Cannot calculate target pose.");
      return null;
    }

    // Calculate offset vector
    Translation2d offsetVector =
        new Translation2d(
            chosen.rotation2D().getCos() * offsetMeters * -1.0, // X-component
            chosen.rotation2D().getSin() * offsetMeters * -1.0 // Y-component
            );

    // Apply offset to the pole translation
    Translation2d adjustedTranslation = poleTranslation.plus(offsetVector);

    // Return the target pose with adjusted translation and chosen rotation
    return new Pose2d(adjustedTranslation, chosen.rotation2D());
  }
}
