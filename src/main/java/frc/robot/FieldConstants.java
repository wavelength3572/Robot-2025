package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.coral.CoralSystemPresets;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class FieldConstants {

  public static final double THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE = 3.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_CAGE = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_DISLODGE = 3.0; // meters

  public static Translation2d selectedCageTranslation = new Translation2d();
  // Assume these are your fixed cage positions defined somewhere (using WPILib's
  // coordinates)
  private static Translation2d BLUE_CAGE_LEFT = new Translation2d(8.72, 7.28);
  private static Translation2d BLUE_CAGE_MID = new Translation2d(8.72, 6.14);
  private static Translation2d BLUE_CAGE_RIGHT = new Translation2d(8.72, 5.09);

  private static Translation2d RED_CAGE_LEFT = new Translation2d(8.72, 3.03);
  private static Translation2d RED_CAGE_MID = new Translation2d(8.72, 1.83);
  private static Translation2d RED_CAGE_RIGHT = new Translation2d(8.72, 0.78);

  public enum CageTarget {
    LEFT,
    MID,
    RIGHT
  }

  public static Translation2d getCage(CageTarget cageTarget) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        switch (cageTarget) {
          case LEFT:
            return BLUE_CAGE_LEFT;

          case MID:
            return BLUE_CAGE_MID;

          case RIGHT:
            return BLUE_CAGE_RIGHT;
        }
      }
      switch (cageTarget) {
        case LEFT:
          return RED_CAGE_LEFT;

        case MID:
          return RED_CAGE_MID;

        case RIGHT:
          return RED_CAGE_RIGHT;
      }
    }
    return null;
  }

  public static final Map<Integer, Translation2d> BLUE_REEF_APRIL_TAGS = new HashMap<>(); // Blue
  public static final Map<Integer, Translation2d> RED_REEF_APRIL_TAGS = new HashMap<>(); // Red

  static {
    // ---------------- BLUE REEF APRIL TAGS ----------------
    BLUE_REEF_APRIL_TAGS.put(17, new Translation2d(4.073905999999999, 3.3063179999999996));
    BLUE_REEF_APRIL_TAGS.put(18, new Translation2d(3.6576, 4.0259)); // 7
    BLUE_REEF_APRIL_TAGS.put(19, new Translation2d(4.073905999999999, 4.745482));
    BLUE_REEF_APRIL_TAGS.put(20, new Translation2d(4.904739999999999, 4.745482));
    BLUE_REEF_APRIL_TAGS.put(21, new Translation2d(5.321046, 4.0259));
    BLUE_REEF_APRIL_TAGS.put(22, new Translation2d(4.904739999999999, 3.3063179999999996));

    // ---------------- RED REEF APRIL TAGS ----------------
    RED_REEF_APRIL_TAGS.put(6, new Translation2d(13.474446, 3.3063179999999996));
    RED_REEF_APRIL_TAGS.put(7, new Translation2d(13.890498, 4.0259));
    RED_REEF_APRIL_TAGS.put(8, new Translation2d(13.47444, 4.745482));
    RED_REEF_APRIL_TAGS.put(9, new Translation2d(12.643358, 4.745482));
    RED_REEF_APRIL_TAGS.put(10, new Translation2d(12.227305999999999, 4.0259));
    RED_REEF_APRIL_TAGS.put(11, new Translation2d(12.643358, 3.3063179999999996));
  }

  public enum ReefOrientationType {
    FRONT,
    BACK
  }

  /**
   * A result that includes both the chosen angle and which orientation (front/back) it came from.
   */
  public record ReefChosenOrientation(Rotation2d rotation2D, ReefOrientationType orientationType) {}

  /** Orientation data for BLUE reef faces, keyed by faceId 17–22. */
  public static final Map<Integer, Rotation2d[]> REEF_FACE_ORIENTATION_BLUE = new HashMap<>();

  /** Orientation data for RED reef faces, keyed by faceId 6–11. */
  public static final Map<Integer, Rotation2d[]> REEF_FACE_ORIENTATION_RED = new HashMap<>();

  static {
    // ---------------- BLUE Orientation ----------------
    REEF_FACE_ORIENTATION_BLUE.put(
        17,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(60)), new Rotation2d(Math.toRadians(60)) // 60, -120
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        18,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(0)), new Rotation2d(Math.toRadians(0))
        }); // 180, -180
    REEF_FACE_ORIENTATION_BLUE.put(
        19,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-60)), new Rotation2d(Math.toRadians(-60)) // 120, -60
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        20,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-120)), new Rotation2d(Math.toRadians(-120))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        21,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(180)), new Rotation2d(Math.toRadians(180))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        22,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(120)), new Rotation2d(Math.toRadians(120))
        });
    Collections.unmodifiableMap(REEF_FACE_ORIENTATION_BLUE);

    // ---------------- RED Orientation ----------------
    REEF_FACE_ORIENTATION_RED.put(
        6,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-60 + 180)), new Rotation2d(Math.toRadians(-60 + 180))
        });
    REEF_FACE_ORIENTATION_RED.put(
        7,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(0 + 180)), new Rotation2d(Math.toRadians(0 + 180))
        });
    REEF_FACE_ORIENTATION_RED.put(
        8,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(60 + 180)), new Rotation2d(Math.toRadians(60 + 180))
        });
    REEF_FACE_ORIENTATION_RED.put(
        9,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(120 + 180)), new Rotation2d(Math.toRadians(120 + 180))
        });
    REEF_FACE_ORIENTATION_RED.put(
        10,
        new Rotation2d[] {new Rotation2d(Math.toRadians(0)), new Rotation2d(Math.toRadians(0))});
    REEF_FACE_ORIENTATION_RED.put(
        11,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(240 - 180)), new Rotation2d(Math.toRadians(240 - 180))
        });
    Collections.unmodifiableMap(REEF_FACE_ORIENTATION_RED);
  }

  // -------------- BLUE BRANCHES -----------------
  public enum ReefBranchesBlue {
    BRANCH_A(new Translation2d(3.2385, 4.3807), new Translation2d(3.2385, 3.9997)),
    BRANCH_B(new Translation2d(3.2385, 4.0521), new Translation2d(3.2385, 3.6711)),
    BRANCH_C(new Translation2d(3.5566, 3.1201), new Translation2d(3.8866, 2.9296)),
    BRANCH_D(new Translation2d(3.7863, 2.9875), new Translation2d(4.1712, 2.7652)),
    BRANCH_E(new Translation2d(4.8075, 2.7652), new Translation2d(5.1374, 2.9557)),
    BRANCH_F(new Translation2d(5.0921, 2.9296), new Translation2d(5.4220, 3.1201)),
    BRANCH_G(new Translation2d(5.7402, 3.6711), new Translation2d(5.7402, 4.0521)),
    BRANCH_H(new Translation2d(5.7402, 3.9997), new Translation2d(5.7402, 4.3807)),
    BRANCH_I(new Translation2d(5.4220, 4.9317), new Translation2d(5.0921, 5.1222)),
    BRANCH_J(new Translation2d(5.1374, 5.0961), new Translation2d(4.8075, 5.2866)),
    BRANCH_K(new Translation2d(4.1712, 5.2866), new Translation2d(3.8412, 5.0961)),
    BRANCH_L(new Translation2d(3.8866, 5.1222), new Translation2d(3.5566, 4.9317));

    private final Translation2d frontTranslation;
    private final Translation2d backTranslation;

    ReefBranchesBlue(Translation2d frontTranslation, Translation2d backTranslation) {
      this.frontTranslation = frontTranslation;
      this.backTranslation = backTranslation;
    }

    public Translation2d getFrontTranslation() {
      return frontTranslation;
    }

    public Translation2d getBackTranslation() {
      return backTranslation;
    }
  }

  public enum ReefFacesBlue {
    FACE_17(17, ReefBranchesBlue.BRANCH_C, ReefBranchesBlue.BRANCH_D),
    FACE_18(18, ReefBranchesBlue.BRANCH_A, ReefBranchesBlue.BRANCH_B),
    FACE_19(19, ReefBranchesBlue.BRANCH_K, ReefBranchesBlue.BRANCH_L),
    FACE_20(20, ReefBranchesBlue.BRANCH_I, ReefBranchesBlue.BRANCH_J),
    FACE_21(21, ReefBranchesBlue.BRANCH_G, ReefBranchesBlue.BRANCH_H),
    FACE_22(22, ReefBranchesBlue.BRANCH_E, ReefBranchesBlue.BRANCH_F);

    private final int faceId;
    private final ReefBranchesBlue leftPole;
    private final ReefBranchesBlue rightPole;

    ReefFacesBlue(int faceId, ReefBranchesBlue leftPole, ReefBranchesBlue rightPole) {
      this.faceId = faceId;
      this.leftPole = leftPole;
      this.rightPole = rightPole;
    }

    public int getFaceId() {
      return faceId;
    }

    public ReefBranchesBlue getLeftPole() {
      return leftPole;
    }

    public ReefBranchesBlue getRightPole() {
      return rightPole;
    }

    /**
     * Find the ReefFaces enum constant matching the given faceId, or return null if none matches.
     */
    public static ReefFacesBlue fromId(int faceId) {
      for (ReefFacesBlue face : values()) {
        if (face.faceId == faceId) {
          return face;
        }
      }
      return null; // or throw an IllegalArgumentException
    }
  }

  public enum ReefBranchesRed {
    // Give each Red branch a matching Blue branch in parentheses
    BRANCH_A(ReefBranchesBlue.BRANCH_B),
    BRANCH_B(ReefBranchesBlue.BRANCH_A),
    BRANCH_C(ReefBranchesBlue.BRANCH_L),
    BRANCH_D(ReefBranchesBlue.BRANCH_K),
    BRANCH_E(ReefBranchesBlue.BRANCH_J),
    BRANCH_F(ReefBranchesBlue.BRANCH_I),
    BRANCH_G(ReefBranchesBlue.BRANCH_H),
    BRANCH_H(ReefBranchesBlue.BRANCH_G),
    BRANCH_I(ReefBranchesBlue.BRANCH_F),
    BRANCH_J(ReefBranchesBlue.BRANCH_E),
    BRANCH_K(ReefBranchesBlue.BRANCH_D),
    BRANCH_L(ReefBranchesBlue.BRANCH_C);

    private final Translation2d frontTranslation;
    private final Translation2d backTranslation;

    /** Constructor takes the matching "blue" branch, then mirrors its front/back translations. */
    ReefBranchesRed(ReefBranchesBlue blueBranch) {
      this.frontTranslation = reflectBlueToRed(blueBranch.getFrontTranslation());
      this.backTranslation = reflectBlueToRed(blueBranch.getBackTranslation());
    }

    public Translation2d getFrontTranslation() {
      return frontTranslation;
    }

    public Translation2d getBackTranslation() {
      return backTranslation;
    }

    // The same mirror logic as above; you can inline it if you prefer
    private static Translation2d reflectBlueToRed(Translation2d blueCoord) {
      double fieldWidth = 17.55;
      return new Translation2d(fieldWidth - blueCoord.getX(), blueCoord.getY());
    }
  }

  public enum ReefFacesRed {
    FACE_6(6, ReefBranchesRed.BRANCH_K, ReefBranchesRed.BRANCH_L),
    FACE_7(7, ReefBranchesRed.BRANCH_A, ReefBranchesRed.BRANCH_B),
    FACE_8(8, ReefBranchesRed.BRANCH_C, ReefBranchesRed.BRANCH_D),
    FACE_9(9, ReefBranchesRed.BRANCH_E, ReefBranchesRed.BRANCH_F),
    FACE_10(10, ReefBranchesRed.BRANCH_G, ReefBranchesRed.BRANCH_H),
    FACE_11(11, ReefBranchesRed.BRANCH_I, ReefBranchesRed.BRANCH_J);

    private final int faceId;
    private final ReefBranchesRed leftPole;
    private final ReefBranchesRed rightPole;

    ReefFacesRed(int faceId, ReefBranchesRed leftPole, ReefBranchesRed rightPole) {
      this.faceId = faceId;
      this.leftPole = leftPole;
      this.rightPole = rightPole;
    }

    public int getFaceId() {
      return faceId;
    }

    public ReefBranchesRed getLeftPole() {
      return leftPole;
    }

    public ReefBranchesRed getRightPole() {
      return rightPole;
    }

    public static ReefFacesRed fromId(int faceId) {
      for (ReefFacesRed face : values()) {
        if (face.faceId == faceId) {
          return face;
        }
      }
      return null; // or throw
    }
  }

  /**
   * Reflects a "blue side" coordinate across the field's vertical center line (x=8.775 on a
   * 17.55m-wide field).
   *
   * @param blueCoord the (x,y) on blue side
   * @return the corresponding (x,y) on red side
   */
  public static Translation2d reflectBlueToRed(Translation2d blueCoord) {
    double fieldWidthX = 17.55; // total field length in X meters
    double xBlue = blueCoord.getX();
    double yBlue = blueCoord.getY();

    double xRed = fieldWidthX - xBlue; // reflect in X
    double yRed = yBlue; // Y stays the same

    return new Translation2d(xRed, yRed);
  }

  public record StationChosenOrientation(
      Rotation2d rotation2D, StationOrientationType orientationType) {}

  public enum StationOrientationType {
    FRONT,
    BACK
  }

  public static final Map<Integer, Translation2d> BLUE_CORALSTATION_APRIL_TAGS =
      new HashMap<>(); // Blue
  public static final Map<Integer, Translation2d> RED_CORALSTATION_APRIL_TAGS =
      new HashMap<>(); // Red

  static {
    // ---------------- BLUE CORAL STATION APRIL TAGS ----------------
    BLUE_CORALSTATION_APRIL_TAGS.put(
        12, new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8)));
    BLUE_CORALSTATION_APRIL_TAGS.put(
        13, new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.2)));

    // ---------------- RED CORAL STATION APRIL TAGS ----------------
    RED_CORALSTATION_APRIL_TAGS.put(
        1, new Translation2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.8)));
    RED_CORALSTATION_APRIL_TAGS.put(
        2, new Translation2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.2)));
  }

  /** Orientation data for BLUE reef faces, keyed by faceId 12-13 */
  public static final Map<Integer, Rotation2d[]> CORAL_STATION_ORIENTATION_BLUE = new HashMap<>();

  /** Orientation data for RED reef faces, keyed by faceId 1-2 */
  public static final Map<Integer, Rotation2d[]> CORAL_STATION_ORIENTATION_RED = new HashMap<>();

  static {
    // ---------------- BLUE Orientation ----------------
    CORAL_STATION_ORIENTATION_BLUE.put(
        12,
        new Rotation2d[] {new Rotation2d(Math.toRadians(54)), new Rotation2d(Math.toRadians(54))});
    CORAL_STATION_ORIENTATION_BLUE.put(
        13,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(306)), new Rotation2d(Math.toRadians(306))
        }); // 180, -180
    Collections.unmodifiableMap(CORAL_STATION_ORIENTATION_BLUE);

    // ---------------- RED Orientation ----------------
    CORAL_STATION_ORIENTATION_RED.put(
        1,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(126)), new Rotation2d(Math.toRadians(126))
        });
    CORAL_STATION_ORIENTATION_RED.put(
        2,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(234)), new Rotation2d(Math.toRadians(234))
        });
    Collections.unmodifiableMap(CORAL_STATION_ORIENTATION_RED);
  }

  public static Pose3d coralPose0 =
      new Pose3d(
          0.720,
          0.478,
          1.175,
          new Rotation3d(0, Units.degreesToRadians(35), Units.degreesToRadians(56)));

  public static Pose3d coralPose0Hidden =
      coralPose0.plus(new Transform3d(0, 0, -10, new Rotation3d()));

  // Map of known staged coral positions (must match JSON!)
  public static final Map<String, Pose3d> stagedCoralPositions =
      Map.of(
          "Coral0", new Pose3d(0.5, 1.0, 1.175, new Rotation3d(0, 0, 0)),
          "Coral1", new Pose3d(1.0, 1.5, 1.175, new Rotation3d(0, 0, 0)),
          "Coral2", new Pose3d(1.5, 2.0, 1.175, new Rotation3d(0, 0, 0)));

  public static final Map<Integer, Pose2d> REEF_FACE_POSES_BLUE;
  public static final Map<Integer, Pose2d> REEF_FACE_POSES_RED;

  static {
    // ----- Build Blue Reef Face Poses -----
    Map<Integer, Pose2d> bluePoses = new HashMap<>();
    for (ReefFacesBlue face : ReefFacesBlue.values()) {
      int blueFaceId = face.getFaceId();
      Translation2d left = face.getLeftPole().getFrontTranslation();
      Translation2d right = face.getRightPole().getFrontTranslation();
      double midX = (left.getX() + right.getX()) / 2.0;
      double midY = (left.getY() + right.getY()) / 2.0;
      Translation2d midpoint = new Translation2d(midX, midY);

      // Retrieve the blue orientation(s) for this face id.
      Rotation2d[] possibleOrients = REEF_FACE_ORIENTATION_BLUE.get(blueFaceId);
      Rotation2d chosenRotation =
          (possibleOrients != null && possibleOrients.length > 0)
              ? possibleOrients[0] // Default to the first orientation.
              : new Rotation2d(); // Fallback to 0 radians.

      bluePoses.put(blueFaceId, new Pose2d(midpoint, chosenRotation));
    }
    REEF_FACE_POSES_BLUE = Collections.unmodifiableMap(bluePoses);

    // ----- Build Red Reef Face Poses -----
    // The mapping is: blue id 17 -> red id 6, 18 -> 7, 19 -> 8, 20 -> 9, 21 -> 10,
    // 22 -> 11.
    Map<Integer, Pose2d> redPoses = new HashMap<>();
    for (Map.Entry<Integer, Pose2d> entry : REEF_FACE_POSES_BLUE.entrySet()) {
      int blueFaceId = entry.getKey();
      // Compute the red face ID by subtracting 11.
      int redFaceId = blueFaceId - 11; // e.g., 17-11 = 6, 18-11 = 7, etc.

      Translation2d blueTranslation = entry.getValue().getTranslation();
      // Reflect the blue translation to get the red translation.
      Translation2d redTranslation = reflectBlueToRed(blueTranslation);

      // Get the red orientation for this face id.
      Rotation2d[] possibleRedOrients = REEF_FACE_ORIENTATION_RED.get(redFaceId);
      Rotation2d redRotation =
          (possibleRedOrients != null && possibleRedOrients.length > 0)
              ? possibleRedOrients[0]
              : entry.getValue().getRotation(); // Fallback: use the blue rotation if none defined.

      redPoses.put(redFaceId, new Pose2d(redTranslation, redRotation));
    }
    REEF_FACE_POSES_RED = Collections.unmodifiableMap(redPoses);
  }

  public static final Set<Integer> DISLODGE_L1_FACES_BLUE = Set.of(17, 19, 21);
  public static final Set<Integer> DISLODGE_L2_FACES_BLUE = Set.of(18, 20, 22);

  public static final Set<Integer> DISLODGE_L1_FACES_RED = Set.of(6, 8, 10);
  public static final Set<Integer> DISLODGE_L2_FACES_RED = Set.of(7, 9, 11);

  /**
   * Returns the appropriate dislodge preset for a given reef face ID based on the current alliance.
   *
   * @param faceId The reef face ID.
   * @return The dislodge preset to use, or null if no mapping is defined.
   */
  public static CoralSystemPresets getDislodgePresetForFace(int faceId) {
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
      if (DISLODGE_L1_FACES_BLUE.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1;
      } else if (DISLODGE_L2_FACES_BLUE.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2;
      }
    } else { // Alliance.Red
      if (DISLODGE_L1_FACES_RED.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1;
      } else if (DISLODGE_L2_FACES_RED.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2;
      }
    }
    return null;
  }

  // ---------------- STAGED ALGAE POSITIONS ----------------
  public static final Map<String, Translation3d> STAGED_ALGAE_POSITIONS;

  static {
    Map<String, Translation3d> algaePositions = new HashMap<>();

    // These values are measured in Onshape (using a centered coordinate system).
    // We immediately convert them to WPILib coordinates (where (0,0,0) is the
    // bottom-left)

    // Blue Side Staged Algae
    algaePositions.put(
        "ALGAE_1",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(7.555, 0.0, 0.497))); // Middle Snowcone Algae
    algaePositions.put(
        "ALGAE_2",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(3.945, 0.588, 1.313))); // Blue 6
    algaePositions.put(
        "ALGAE_3",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(4.624, 0.588, 0.909))); // Blue 5
    algaePositions.put(
        "ALGAE_4",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(3.605, 0.0, 0.909))); // Blue 4
    algaePositions.put(
        "ALGAE_5",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(4.624, -0.588, 0.909))); // Blue 2
    algaePositions.put(
        "ALGAE_6",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(3.945, -0.588, 1.313))); // Blue 3
    algaePositions.put(
        "ALGAE_7",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(4.964, 0.0, 1.313))); // Blue 1
    algaePositions.put(
        "ALGAE_8",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(7.555, 1.829, 0.497))); // Right Snowcone Algae
    algaePositions.put(
        "ALGAE_9",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(7.555, -1.829, 0.497))); // Left Snowcone Algae

    // Red Side Staged Algae
    algaePositions.put(
        "ALGAE_10",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-7.555, 0.0, 0.497))); // Middle Snowcone Algae
    algaePositions.put(
        "ALGAE_11",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-7.555, -1.829, 0.497))); // Right Snowcone Algae
    algaePositions.put(
        "ALGAE_12",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-7.555, 1.829, 0.497))); // Left Snowcone Algae
    algaePositions.put(
        "ALGAE_13",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-4.964, 0.0, 1.313))); // Red 1
    algaePositions.put(
        "ALGAE_14",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-3.945, 0.588, 1.313))); // Red 3
    algaePositions.put(
        "ALGAE_15",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-4.624, 0.588, 0.909))); // Red 2
    algaePositions.put(
        "ALGAE_16",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-3.605, 0.0, 0.909))); // Red 4
    algaePositions.put(
        "ALGAE_18",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-3.945, -0.588, 1.313))); // Red 5
    algaePositions.put(
        "ALGAE_19",
        convertOnShapeOriginMeasurementToWPILibCoordinates(
            new Translation3d(-4.624, -0.588, 0.909))); // Red 6

    STAGED_ALGAE_POSITIONS = Collections.unmodifiableMap(algaePositions);
  }

  /** Retrieves the original algae position from its name (in WPILib coordinates). */
  public static Translation3d getStagedAlgaePosition(String algaeName) {
    return STAGED_ALGAE_POSITIONS.getOrDefault(algaeName, new Translation3d(0, 0, 0));
  }

  /**
   * Retrieves all staged algae positions as an array for logging (already in WPILib coordinates).
   */
  public static Translation3d[] getAllStagedAlgaePositions() {
    return STAGED_ALGAE_POSITIONS.values().toArray(new Translation3d[0]);
  }

  /**
   * Converts an Onshape-centered position (with (0,0,0) at the field center) to WPILib coordinates,
   * where (0,0,0) is the bottom-left of the field.
   *
   * <p>This method assumes the field is 17.55m long and 8.05m wide.
   */
  public static Translation3d convertOnShapeOriginMeasurementToWPILibCoordinates(
      Translation3d onshapePos) {
    double fieldLength = 17.55; // Field length in meters
    double fieldWidth = 8.05; // Field width in meters
    double halfLength = fieldLength / 2.0; // Center of the field in X (approx. 8.775)
    double halfWidth = fieldWidth / 2.0; // Center of the field in Y (approx. 4.025)

    // Since the Onshape measurements are taken from a centered coordinate system,
    // to convert to WPILib (bottom-left origin), add halfLength to the X value
    // and halfWidth to the Y value, then invert the sign if necessary.
    // Here we assume that increasing X in WPILib goes right, and increasing Y goes
    // up.
    return new Translation3d(
        -onshapePos.getX() + halfLength,
        -onshapePos.getY() + halfWidth,
        onshapePos.getZ() // Z remains unchanged
        );
  }
}
