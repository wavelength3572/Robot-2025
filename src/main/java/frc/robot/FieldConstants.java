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

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

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



  // Common Reef Geometry (Onshape-centered)
  public static final Translation3d REEF_CENTER_ONSHAPE = new Translation3d(4.280, 0, 0);
  public static final double POLE_OFFSET = 0.165; // offset along the face's local Y-axis

  // L1 Coral Constants
  public static final double L1_EFFECTIVE_RADIUS = 0.78625;
  public static final double L1_HEIGHT = 0.5; // L1 coral height in meters
  public static final double L1_FIXED_PITCH_DEG = 0; // L1 coral pitch (degrees)
  public static final double L1_BASE_ANGLE_DEG = 0; // Face1's center is along 0° in Onshape
  public static final double L1_YAW_OFFSET_DEG = 90; // Additional yaw offset for L1 coral

  // L2 Coral Constants
  public static final double L2_EFFECTIVE_RADIUS = 0.71;
  public static final double L2_HEIGHT = 0.72; // L2 coral height in meters
  public static final double L2_FIXED_PITCH_DEG = -34; // L2 coral fixed pitch (degrees)
  public static final double L2_BASE_ANGLE_DEG = 0; // Face1's center is along 0° in Onshape

  // L3 Coral Constants
  public static final double L3_EFFECTIVE_RADIUS = 0.71;
  public static final double L3_HEIGHT = 1.13; // L3 coral height in meters
  public static final double L3_FIXED_PITCH_DEG = -34; // L3 coral fixed pitch (degrees)
  public static final double L3_BASE_ANGLE_DEG = 0; // Face1's center is along 0° in Onshape

  // ---------------- L4 Coral Constants ----------------
  public static final double L4_EFFECTIVE_RADIUS = 0.78025;
  public static final double L4_HEIGHT = 1.745; // L4 coral height (m)
  public static final double L4_FIXED_PITCH_DEG = 90; // L4 coral fixed pitch (deg)
  public static final double L4_BASE_ANGLE_DEG = 0; // Face1's center angle (deg)

  static {
    // Generate red and blue coral poses for each level.
    Pose3d[] redScoredCoralsL1 =
        generateScoredCoralPosesL1(
            REEF_CENTER_ONSHAPE,
            L1_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L1_HEIGHT,
            L1_FIXED_PITCH_DEG,
            L1_BASE_ANGLE_DEG,
            L1_YAW_OFFSET_DEG);
    Pose3d[] redScoredCoralsL2 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L2_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L2_HEIGHT,
            L2_FIXED_PITCH_DEG,
            L2_BASE_ANGLE_DEG);
    Pose3d[] redScoredCoralsL3 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L3_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L3_HEIGHT,
            L3_FIXED_PITCH_DEG,
            L3_BASE_ANGLE_DEG);
    Pose3d[] redScoredCoralsL4 =
        generateScoredCoralPosesL4(
            REEF_CENTER_ONSHAPE,
            L4_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L4_HEIGHT,
            L4_FIXED_PITCH_DEG,
            L4_BASE_ANGLE_DEG);

    Pose3d[] blueScoredCoralsL1 = reflectPoseArray(redScoredCoralsL1);
    Pose3d[] blueScoredCoralsL2 = reflectPoseArray(redScoredCoralsL2);
    Pose3d[] blueScoredCoralsL3 = reflectPoseArray(redScoredCoralsL3);
    Pose3d[] blueScoredCoralsL4 = reflectPoseArray(redScoredCoralsL4);

    Logger.recordOutput("RedScoredCoralsL1", redScoredCoralsL1);
    Logger.recordOutput("BlueScoredCoralsL1", blueScoredCoralsL1);
    Logger.recordOutput("RedScoredCoralsL2", redScoredCoralsL2);
    Logger.recordOutput("BlueScoredCoralsL2", blueScoredCoralsL2);
    Logger.recordOutput("RedScoredCoralsL3", redScoredCoralsL3);
    Logger.recordOutput("BlueScoredCoralsL3", blueScoredCoralsL3);
    Logger.recordOutput("RedScoredCoralsL4", redScoredCoralsL4);
    Logger.recordOutput("BlueScoredCoralsL4", blueScoredCoralsL4);
  }

  /**
   * Returns a map containing coral poses for each level (L1-L4) for both the red and blue sides.
   * The keys in the map will be "RedL1", "BlueL1", "RedL2", etc.
   */
  public static Map<String, Pose3d[]> getAllCoralPoses() {
    Map<String, Pose3d[]> coralPoses = new HashMap<>();

    // Generate red-side coral poses using your generation methods and constants.
    Pose3d[] redL1 =
        generateScoredCoralPosesL1(
            REEF_CENTER_ONSHAPE,
            L1_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L1_HEIGHT,
            L1_FIXED_PITCH_DEG,
            L1_BASE_ANGLE_DEG,
            L1_YAW_OFFSET_DEG);
    Pose3d[] redL2 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L2_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L2_HEIGHT,
            L2_FIXED_PITCH_DEG,
            L2_BASE_ANGLE_DEG);
    Pose3d[] redL3 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L3_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L3_HEIGHT,
            L3_FIXED_PITCH_DEG,
            L3_BASE_ANGLE_DEG);
    Pose3d[] redL4 =
        generateScoredCoralPosesL4(
            REEF_CENTER_ONSHAPE,
            L4_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L4_HEIGHT,
            L4_FIXED_PITCH_DEG,
            L4_BASE_ANGLE_DEG);

    // Reflect red poses to generate blue poses.
    Pose3d[] blueL1 = generateBlueCoralPosesFromRed(redL1);
    Pose3d[] blueL2 = generateBlueCoralPosesFromRed(redL2);
    Pose3d[] blueL3 = generateBlueCoralPosesFromRed(redL3);
    Pose3d[] blueL4 = generateBlueCoralPosesFromRed(redL4);

    // Put them in the map.
    coralPoses.put("RedL1", redL1);
    coralPoses.put("BlueL1", blueL1);
    coralPoses.put("RedL2", redL2);
    coralPoses.put("BlueL2", blueL2);
    coralPoses.put("RedL3", redL3);
    coralPoses.put("BlueL3", blueL3);
    coralPoses.put("RedL4", redL4);
    coralPoses.put("BlueL4", blueL4);

    return coralPoses;
  }

  /**
   * Returns an array of 12 scored coral Pose3d objects for L4. This method is fully parameterized,
   * so you can supply the reef geometry and rotation parameters.
   *
   * @param reefCenterOnshape the reef center in Onshape-centered coordinates.
   * @param effectiveRadius the effective radius (distance from reef center to a face center).
   * @param poleOffset the offset along the face's local Y-axis for the coral poles.
   * @param fixedHeight the fixed L4 coral height.
   * @param fixedPitchDeg the fixed pitch angle (in degrees) for all coral.
   * @param baseAngleDeg the angle (in degrees) of Face1’s center relative to the reef center.
   * @return an array of 12 Pose3d objects (one for each reef pole) in WPILib coordinates.
   */
  public static Pose3d[] generateScoredCoralPosesL4(
      Translation3d reefCenterOnshape,
      double effectiveRadius,
      double poleOffset,
      double fixedHeight,
      double fixedPitchDeg,
      double baseAngleDeg) {
    Pose3d[] poses = new Pose3d[12];
    int index = 0;

    // Convert base angle to radians.
    double baseAngleRad = Math.toRadians(baseAngleDeg);

    // There are 6 faces (each yielding 2 poles → 12 total), evenly spaced (60°
    // apart).
    for (int i = 0; i < 6; i++) {
      // Compute the face center angle in Onshape coordinates.
      double faceAngle = baseAngleRad + i * Math.toRadians(60);
      // Compute the face center from the reef center.
      double faceCenterX = reefCenterOnshape.getX() + effectiveRadius * Math.cos(faceAngle);
      double faceCenterY = reefCenterOnshape.getY() + effectiveRadius * Math.sin(faceAngle);
      Translation3d faceCenter = new Translation3d(faceCenterX, faceCenterY, 0);

      // Compute the two coral poles for this face by offsetting the face center along
      // the local Y-axis.
      Translation3d localOffsetA = new Translation3d(0, -poleOffset, 0);
      Translation3d localOffsetB = new Translation3d(0, poleOffset, 0);
      Translation3d rotatedOffsetA = rotateTranslationByAngle(localOffsetA, faceAngle);
      Translation3d rotatedOffsetB = rotateTranslationByAngle(localOffsetB, faceAngle);

      Translation3d poleAOnshape = faceCenter.plus(rotatedOffsetA);
      Translation3d poleBOnshape = faceCenter.plus(rotatedOffsetB);
      // Set the fixed L4 height.
      poleAOnshape = new Translation3d(poleAOnshape.getX(), poleAOnshape.getY(), fixedHeight);
      poleBOnshape = new Translation3d(poleBOnshape.getX(), poleBOnshape.getY(), fixedHeight);

      // Convert these Onshape-centered positions to WPILib coordinates.
      Translation3d poleAWP = convertCenteredToWPILib(poleAOnshape);
      Translation3d poleBWP = convertCenteredToWPILib(poleBOnshape);

      // Compute final rotation.
      // We use a fixed pitch (converted to radians) and set yaw equal to the face
      // angle.
      Rotation3d fixedRotation =
          new Rotation3d(0, Units.degreesToRadians(fixedPitchDeg), faceAngle);

      poses[index++] = new Pose3d(poleAWP, fixedRotation);
      poses[index++] = new Pose3d(poleBWP, fixedRotation);
    }

    return poses;
  }

  /**
   * Converts an Onshape-centered position (with (0,0,0) at the field center) to WPILib coordinates,
   * where (0,0,0) is the bottom-left of the field. Assumes field dimensions: length = 17.55 m,
   * width = 8.05 m.
   */
  public static Translation3d convertCenteredToWPILib(Translation3d onshapePos) {
    double fieldLength = 17.55;
    double fieldWidth = 8.05;
    double halfLength = fieldLength / 2.0; // ~8.775 m
    double halfWidth = fieldWidth / 2.0; // ~4.025 m (adjust if necessary)

    return new Translation3d(
        onshapePos.getX() + halfLength, onshapePos.getY() + halfWidth, onshapePos.getZ());
  }

  /**
   * Rotates a Translation3d about the Z-axis by the given angle (in radians). Only rotates X and Y;
   * Z remains unchanged.
   */
  public static Translation3d rotateTranslationByAngle(Translation3d translation, double angleRad) {
    double x = translation.getX();
    double y = translation.getY();
    double rotatedX = x * Math.cos(angleRad) - y * Math.sin(angleRad);
    double rotatedY = x * Math.sin(angleRad) + y * Math.cos(angleRad);
    return new Translation3d(rotatedX, rotatedY, translation.getZ());
  }

  /**
   * Generates an array of 12 scored coral Pose3d objects for L3 using a parameterized reef
   * geometry.
   *
   * @param reefCenterOnshape the reef center in Onshape-centered coordinates.
   * @param effectiveRadius the effective radius (distance from reef center to a face center).
   * @param poleOffset the offset along the face's local Y-axis for the coral poles.
   * @param fixedHeight the fixed height (L3 height) for the coral.
   * @param fixedPitchDeg the fixed pitch angle (in degrees) to apply to all coral.
   * @param baseAngleDeg the angle (in degrees) of Face1’s center relative to the reef center.
   * @return an array of 12 Pose3d objects (one for each reef pole) in WPILib coordinates.
   */
  public static Pose3d[] generateScoredCoralPosesAngled(
      Translation3d reefCenterOnshape,
      double effectiveRadius,
      double poleOffset,
      double fixedHeight,
      double fixedPitchDeg,
      double baseAngleDeg) {
    Pose3d[] poses = new Pose3d[12];
    int index = 0;

    // Convert base angle to radians.
    double baseAngleRad = Math.toRadians(baseAngleDeg);

    // There are 6 faces (each face yields 2 poles → 12 total), evenly spaced (60°
    // apart).
    for (int i = 0; i < 6; i++) {
      // Compute face center angle in Onshape coordinates.
      double faceAngle = baseAngleRad + i * Math.toRadians(60);
      // Compute the face center from the reef center.
      double faceCenterX = reefCenterOnshape.getX() + effectiveRadius * Math.cos(faceAngle);
      double faceCenterY = reefCenterOnshape.getY() + effectiveRadius * Math.sin(faceAngle);
      Translation3d faceCenter = new Translation3d(faceCenterX, faceCenterY, 0);

      // For each face, the two coral poles are offset by ±poleOffset along the face's
      // local Y-axis.
      Translation3d localOffsetA = new Translation3d(0, -poleOffset, 0);
      Translation3d localOffsetB = new Translation3d(0, poleOffset, 0);
      Translation3d rotatedOffsetA = rotateTranslationByAngle(localOffsetA, faceAngle);
      Translation3d rotatedOffsetB = rotateTranslationByAngle(localOffsetB, faceAngle);

      // Compute the Onshape coordinates for the two poles.
      Translation3d poleAOnshape = faceCenter.plus(rotatedOffsetA);
      Translation3d poleBOnshape = faceCenter.plus(rotatedOffsetB);
      // Set fixed height.
      poleAOnshape = new Translation3d(poleAOnshape.getX(), poleAOnshape.getY(), fixedHeight);
      poleBOnshape = new Translation3d(poleBOnshape.getX(), poleBOnshape.getY(), fixedHeight);

      // Convert these Onshape-centered coordinates to WPILib coordinates.
      Translation3d poleAWP = convertCenteredToWPILib(poleAOnshape);
      Translation3d poleBWP = convertCenteredToWPILib(poleBOnshape);

      // For rotation, use a fixed pitch (fixedPitchDeg) and set yaw equal to the
      // face’s angle.
      Rotation3d fixedRotation =
          new Rotation3d(0, Units.degreesToRadians(fixedPitchDeg), faceAngle);

      poses[index++] = new Pose3d(poleAWP, fixedRotation);
      poses[index++] = new Pose3d(poleBWP, fixedRotation);
    }

    return poses;
  }

  /**
   * Adjusts a coral position by moving it radially toward the reef center by a specified distance
   * and lowering its Z coordinate by a specified amount.
   *
   * @param coralWP the coral position in WPILib coordinates.
   * @param reefCenterWP the reef center in WPILib coordinates.
   * @param radialShift the distance to move toward the reef center (in meters).
   * @param zShift the distance to lower the coral (in meters).
   * @return the adjusted coral position.
   */
  public static Translation3d adjustCoralPosition(
      Translation3d coralWP, Translation3d reefCenterWP, double radialShift, double zShift) {
    double dx = reefCenterWP.getX() - coralWP.getX();
    double dy = reefCenterWP.getY() - coralWP.getY();
    double distance = Math.hypot(dx, dy);
    // Prevent division by zero.
    if (distance == 0) {
      return new Translation3d(coralWP.getX(), coralWP.getY(), coralWP.getZ() - zShift);
    }
    double normX = dx / distance;
    double normY = dy / distance;
    // Move the coral toward the reef center by radialShift, and lower by zShift.
    return new Translation3d(
        coralWP.getX() + radialShift * normX,
        coralWP.getY() + radialShift * normY,
        coralWP.getZ() - zShift);
  }

  /**
   * Generates an array of 12 scored coral Pose3d objects for L1, allowing you to specify an
   * additional yaw offset.
   *
   * @param reefCenterOnshape the reef center in Onshape-centered coordinates.
   * @param effectiveRadius the effective radius (distance from reef center to a face center).
   * @param poleOffset the offset along the face's local Y-axis for the coral poles.
   * @param fixedHeight the fixed L1 height for the coral.
   * @param fixedPitchDeg the fixed pitch angle (in degrees) for the coral.
   * @param baseAngleDeg the angle (in degrees) of Face1’s center relative to the reef center.
   * @param yawOffsetDeg the additional yaw offset (in degrees) to apply to each coral.
   * @return an array of 12 Pose3d objects (one for each reef pole) in WPILib coordinates.
   */
  public static Pose3d[] generateScoredCoralPosesL1(
      Translation3d reefCenterOnshape,
      double effectiveRadius,
      double poleOffset,
      double fixedHeight,
      double fixedPitchDeg,
      double baseAngleDeg,
      double yawOffsetDeg) {
    Pose3d[] poses = new Pose3d[12];
    int index = 0;
    double baseAngleRad = Math.toRadians(baseAngleDeg);

    for (int i = 0; i < 6; i++) {
      double faceAngle = baseAngleRad + i * Math.toRadians(60);
      double faceCenterX = reefCenterOnshape.getX() + effectiveRadius * Math.cos(faceAngle);
      double faceCenterY = reefCenterOnshape.getY() + effectiveRadius * Math.sin(faceAngle);
      Translation3d faceCenter = new Translation3d(faceCenterX, faceCenterY, 0);

      Translation3d localOffsetA = new Translation3d(0, -poleOffset, 0);
      Translation3d localOffsetB = new Translation3d(0, poleOffset, 0);
      Translation3d rotatedOffsetA = rotateTranslationByAngle(localOffsetA, faceAngle);
      Translation3d rotatedOffsetB = rotateTranslationByAngle(localOffsetB, faceAngle);

      Translation3d poleAOnshape = faceCenter.plus(rotatedOffsetA);
      Translation3d poleBOnshape = faceCenter.plus(rotatedOffsetB);
      poleAOnshape = new Translation3d(poleAOnshape.getX(), poleAOnshape.getY(), fixedHeight);
      poleBOnshape = new Translation3d(poleBOnshape.getX(), poleBOnshape.getY(), fixedHeight);

      Translation3d poleAWP = convertCenteredToWPILib(poleAOnshape);
      Translation3d poleBWP = convertCenteredToWPILib(poleBOnshape);

      Rotation3d coralRotation =
          new Rotation3d(
              0, Units.degreesToRadians(fixedPitchDeg), faceAngle + Math.toRadians(yawOffsetDeg));

      poses[index++] = new Pose3d(poleAWP, coralRotation);
      poses[index++] = new Pose3d(poleBWP, coralRotation);
    }
    return poses;
  }

  /**
   * Subtracts the reference position from pos and rotates the result by angleRad. This effectively
   * transforms pos into a coordinate system with origin at ref and rotated by angleRad.
   */
  public static Translation3d subtractAndRotate(
      Translation3d pos, Translation3d ref, double angleRad) {
    double dx = pos.getX() - ref.getX();
    double dy = pos.getY() - ref.getY();
    double rotatedX = dx * Math.cos(angleRad) - dy * Math.sin(angleRad);
    double rotatedY = dx * Math.sin(angleRad) + dy * Math.cos(angleRad);
    return new Translation3d(rotatedX, rotatedY, pos.getZ()); // Z is unchanged
  }

  public static Pose3d[] reflectPoseArray(Pose3d[] poses) {
    Pose3d[] reflected = new Pose3d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      reflected[i] = reflectPoseAcrossMidline(poses[i]);
    }
    return reflected;
  }

  /**
   * Reflects an individual pose across the vertical midline. (This is your existing reflection
   * function.)
   */
  public static Pose3d reflectPoseAcrossMidline(Pose3d pose) {
    double fieldLength = 17.55;
    double midX = fieldLength / 2.0; // ~8.775 m
    Translation3d t = pose.getTranslation();
    double reflectedX = 2 * midX - t.getX();
    Translation3d newTrans = new Translation3d(reflectedX, t.getY(), t.getZ());
    // For rotation, assume a simple Euler (roll, pitch, yaw)
    double roll = pose.getRotation().getX();
    double pitch = pose.getRotation().getY();
    double yaw = pose.getRotation().getZ();
    double newYaw = Math.PI - yaw; // current reflection formula
    Rotation3d newRot = new Rotation3d(roll, pitch, newYaw);
    return new Pose3d(newTrans, newRot);
  }

  public static class CoralKey {
    public final String side; // "Red" or "Blue"
    public final String level; // e.g. "L1", "L2", etc.
    public final int face; // 1 through 6
    public final String pole; // "A" or "B"

    public CoralKey(String side, String level, int face, String pole) {
      this.side = side;
      this.level = level;
      this.face = face;
      this.pole = pole;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof CoralKey)) return false;
      CoralKey key = (CoralKey) o;
      return face == key.face
          && side.equals(key.side)
          && level.equals(key.level)
          && pole.equals(key.pole);
    }

    @Override
    public int hashCode() {
      int result = side.hashCode();
      result = 31 * result + level.hashCode();
      result = 31 * result + face;
      result = 31 * result + pole.hashCode();
      return result;
    }

    @Override
    public String toString() {
      return side + "_" + level + "_" + face + "_" + pole;
    }
  }

  // Simple container for a coral location (the Pose3d plus a flag).
  public static class CoralLocation {
    public final Pose3d pose;
    public boolean scored;

    public CoralLocation(Pose3d pose) {
      this.pose = pose;
      this.scored = false;
    }

    @Override
    public String toString() {
      return "CoralLocation{" + "pose=" + pose + ", scored=" + scored + "}";
    }
  }

  public static final Map<CoralKey, CoralLocation> coralMapping = new HashMap<>();

  static {
    // Build red coral mapping for L1
    Pose3d[] redL1 =
        generateScoredCoralPosesL1(
            REEF_CENTER_ONSHAPE,
            L1_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L1_HEIGHT,
            L1_FIXED_PITCH_DEG,
            L1_BASE_ANGLE_DEG,
            L1_YAW_OFFSET_DEG);
    coralMapping.put(new CoralKey("Red", "L1", 1, "A"), new CoralLocation(redL1[0]));
    coralMapping.put(new CoralKey("Red", "L1", 1, "B"), new CoralLocation(redL1[1]));
    coralMapping.put(new CoralKey("Red", "L1", 2, "A"), new CoralLocation(redL1[10]));
    coralMapping.put(new CoralKey("Red", "L1", 2, "B"), new CoralLocation(redL1[11]));
    coralMapping.put(new CoralKey("Red", "L1", 3, "A"), new CoralLocation(redL1[8]));
    coralMapping.put(new CoralKey("Red", "L1", 3, "B"), new CoralLocation(redL1[9]));
    coralMapping.put(new CoralKey("Red", "L1", 4, "A"), new CoralLocation(redL1[6]));
    coralMapping.put(new CoralKey("Red", "L1", 4, "B"), new CoralLocation(redL1[7]));
    coralMapping.put(new CoralKey("Red", "L1", 5, "A"), new CoralLocation(redL1[4]));
    coralMapping.put(new CoralKey("Red", "L1", 5, "B"), new CoralLocation(redL1[5]));
    coralMapping.put(new CoralKey("Red", "L1", 6, "A"), new CoralLocation(redL1[2]));
    coralMapping.put(new CoralKey("Red", "L1", 6, "B"), new CoralLocation(redL1[3]));

    // Similarly build red coral mapping for L2, L3, and L4…
    Pose3d[] redL2 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L2_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L2_HEIGHT,
            L2_FIXED_PITCH_DEG,
            L2_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey("Red", "L2", 1, "A"), new CoralLocation(redL2[0]));
    coralMapping.put(new CoralKey("Red", "L2", 1, "B"), new CoralLocation(redL2[1]));
    coralMapping.put(new CoralKey("Red", "L2", 2, "A"), new CoralLocation(redL2[10]));
    coralMapping.put(new CoralKey("Red", "L2", 2, "B"), new CoralLocation(redL2[11]));
    coralMapping.put(new CoralKey("Red", "L2", 3, "A"), new CoralLocation(redL2[8]));
    coralMapping.put(new CoralKey("Red", "L2", 3, "B"), new CoralLocation(redL2[9]));
    coralMapping.put(new CoralKey("Red", "L2", 4, "A"), new CoralLocation(redL2[6]));
    coralMapping.put(new CoralKey("Red", "L2", 4, "B"), new CoralLocation(redL2[7]));
    coralMapping.put(new CoralKey("Red", "L2", 5, "A"), new CoralLocation(redL2[4]));
    coralMapping.put(new CoralKey("Red", "L2", 5, "B"), new CoralLocation(redL2[5]));
    coralMapping.put(new CoralKey("Red", "L2", 6, "A"), new CoralLocation(redL2[2]));
    coralMapping.put(new CoralKey("Red", "L2", 6, "B"), new CoralLocation(redL2[3]));

    Pose3d[] redL3 =
        generateScoredCoralPosesAngled(
            REEF_CENTER_ONSHAPE,
            L3_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L3_HEIGHT,
            L3_FIXED_PITCH_DEG,
            L3_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey("Red", "L3", 1, "A"), new CoralLocation(redL3[0]));
    coralMapping.put(new CoralKey("Red", "L3", 1, "B"), new CoralLocation(redL3[1]));
    coralMapping.put(new CoralKey("Red", "L3", 2, "A"), new CoralLocation(redL3[10]));
    coralMapping.put(new CoralKey("Red", "L3", 2, "B"), new CoralLocation(redL3[11]));
    coralMapping.put(new CoralKey("Red", "L3", 3, "A"), new CoralLocation(redL3[8]));
    coralMapping.put(new CoralKey("Red", "L3", 3, "B"), new CoralLocation(redL3[9]));
    coralMapping.put(new CoralKey("Red", "L3", 4, "A"), new CoralLocation(redL3[6]));
    coralMapping.put(new CoralKey("Red", "L3", 4, "B"), new CoralLocation(redL3[7]));
    coralMapping.put(new CoralKey("Red", "L3", 5, "A"), new CoralLocation(redL3[4]));
    coralMapping.put(new CoralKey("Red", "L3", 5, "B"), new CoralLocation(redL3[5]));
    coralMapping.put(new CoralKey("Red", "L3", 6, "A"), new CoralLocation(redL3[2]));
    coralMapping.put(new CoralKey("Red", "L3", 6, "B"), new CoralLocation(redL3[3]));

    Pose3d[] redL4 =
        generateScoredCoralPosesL4(
            REEF_CENTER_ONSHAPE,
            L4_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L4_HEIGHT,
            L4_FIXED_PITCH_DEG,
            L4_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey("Red", "L4", 1, "A"), new CoralLocation(redL4[0]));
    coralMapping.put(new CoralKey("Red", "L4", 1, "B"), new CoralLocation(redL4[1]));
    coralMapping.put(new CoralKey("Red", "L4", 2, "A"), new CoralLocation(redL4[10]));
    coralMapping.put(new CoralKey("Red", "L4", 2, "B"), new CoralLocation(redL4[11]));
    coralMapping.put(new CoralKey("Red", "L4", 3, "A"), new CoralLocation(redL4[8]));
    coralMapping.put(new CoralKey("Red", "L4", 3, "B"), new CoralLocation(redL4[9]));
    coralMapping.put(new CoralKey("Red", "L4", 4, "A"), new CoralLocation(redL4[6]));
    coralMapping.put(new CoralKey("Red", "L4", 4, "B"), new CoralLocation(redL4[7]));
    coralMapping.put(new CoralKey("Red", "L4", 5, "A"), new CoralLocation(redL4[4]));
    coralMapping.put(new CoralKey("Red", "L4", 5, "B"), new CoralLocation(redL4[5]));
    coralMapping.put(new CoralKey("Red", "L4", 6, "A"), new CoralLocation(redL4[2]));
    coralMapping.put(new CoralKey("Red", "L4", 6, "B"), new CoralLocation(redL4[3]));

    // Now generate blue mapping by reflecting the red ones.
    Map<CoralKey, CoralLocation> blueMapping = new HashMap<>();
    for (Map.Entry<CoralKey, CoralLocation> entry : coralMapping.entrySet()) {
      CoralKey redKey = entry.getKey();
      if (redKey.side.equals("Red")) {
        CoralKey blueKey = new CoralKey("Blue", redKey.level, redKey.face, redKey.pole);
        Pose3d bluePose = reflectBluePose(entry.getValue().pose);
        blueMapping.put(blueKey, new CoralLocation(bluePose));
      }
    }
    coralMapping.putAll(blueMapping);
  }

  public static void testScoreFixedCorals() {
    // Fixed selection for Red side coral spots.
    CoralKey redKey1 = new CoralKey("Red", "L1", 1, "A");
    CoralKey redKey2 = new CoralKey("Red", "L2", 3, "B");
    CoralKey redKey3 = new CoralKey("Red", "L3", 5, "A");
    CoralKey redKey4 = new CoralKey("Red", "L4", 2, "B");

    // Fixed selection for Blue side coral spots.
    CoralKey blueKey1 = new CoralKey("Blue", "L1", 2, "A");
    CoralKey blueKey2 = new CoralKey("Blue", "L2", 4, "B");
    CoralKey blueKey3 = new CoralKey("Blue", "L3", 6, "A");
    CoralKey blueKey4 = new CoralKey("Blue", "L4", 3, "B");

    // Mark the selected coral positions as scored.
    if (coralMapping.containsKey(redKey1)) coralMapping.get(redKey1).scored = true;
    if (coralMapping.containsKey(redKey2)) coralMapping.get(redKey2).scored = true;
    if (coralMapping.containsKey(redKey3)) coralMapping.get(redKey3).scored = true;
    if (coralMapping.containsKey(redKey4)) coralMapping.get(redKey4).scored = true;
    if (coralMapping.containsKey(blueKey1)) coralMapping.get(blueKey1).scored = true;
    if (coralMapping.containsKey(blueKey2)) coralMapping.get(blueKey2).scored = true;
    if (coralMapping.containsKey(blueKey3)) coralMapping.get(blueKey3).scored = true;
    if (coralMapping.containsKey(blueKey4)) coralMapping.get(blueKey4).scored = true;

    // Now, collect all scored coral poses.
    List<Pose3d> scoredList = new ArrayList<>();
    for (Map.Entry<CoralKey, CoralLocation> entry : coralMapping.entrySet()) {
      if (entry.getValue().scored) {
        scoredList.add(entry.getValue().pose);
      }
    }

    // Log the scored coral poses so you can inspect them.
    Logger.recordOutput("Test/FixedScoredCorals", scoredList.toArray(new Pose3d[0]));
  }

  static {
    // ... build your coralMapping for L1, L2, L3, and L4 ...

    // Call the test method to mark some coral as scored and log them.
    testScoreFixedCorals();
  }

  /**
   * Given an array of red coral poses (ordered by face 1 … face 6, each with two poses), this
   * method reflects each pose and reorders them so that the blue side faces come out in the
   * expected order.
   *
   * <p>For example, if you find that red face2 (indices 2–3) should correspond to blue face6, red
   * face3 to blue face5, red face4 to blue face4, red face5 to blue face3, and red face6 to blue
   * face2, we can compute a new ordering.
   */
  public static Pose3d[] generateBlueCoralPosesFromRed(Pose3d[] redPoses) {
    // redPoses is assumed to be length 12: face1 (0-1), face2 (2-3), face3 (4-5),
    // face4 (6-7), face5 (8-9), face6 (10-11).
    Pose3d[] bluePoses = new Pose3d[redPoses.length];
    // For face 1, we keep the same order.
    bluePoses[0] = reflectPoseAcrossMidline(redPoses[0]);
    bluePoses[1] = reflectPoseAcrossMidline(redPoses[1]);
    // For faces 2-6, we reverse the order:
    // red face 2 (indices 2-3) should become blue face 6 (indices 10-11)
    bluePoses[10] = reflectPoseAcrossMidline(redPoses[2]);
    bluePoses[11] = reflectPoseAcrossMidline(redPoses[3]);
    // red face 3 (indices 4-5) -> blue face 5 (indices 8-9)
    bluePoses[8] = reflectPoseAcrossMidline(redPoses[4]);
    bluePoses[9] = reflectPoseAcrossMidline(redPoses[5]);
    // red face 4 (indices 6-7) -> blue face 4 (indices 6-7) (unchanged)
    bluePoses[6] = reflectPoseAcrossMidline(redPoses[6]);
    bluePoses[7] = reflectPoseAcrossMidline(redPoses[7]);
    // red face 5 (indices 8-9) -> blue face 3 (indices 4-5)
    bluePoses[4] = reflectPoseAcrossMidline(redPoses[8]);
    bluePoses[5] = reflectPoseAcrossMidline(redPoses[9]);
    // red face 6 (indices 10-11) -> blue face 2 (indices 2-3)
    bluePoses[2] = reflectPoseAcrossMidline(redPoses[10]);
    bluePoses[3] = reflectPoseAcrossMidline(redPoses[11]);

    return bluePoses;
  }

  /**
   * Reflects a red coral Pose3d to the blue side by mirroring it across the field’s vertical
   * midline. Assumes WPILib coordinates with (0,0) at the bottom-left and a field length of 17.55
   * m. For a pose with yaw θ, the reflected yaw is computed as π – θ.
   */
  public static Pose3d reflectBluePose(Pose3d redPose) {
    double fieldLength = 17.55;
    double midX = fieldLength / 2.0; // ~8.775 m

    // Reflect the translation: newX = 2 * midX - oldX, Y and Z remain the same.
    Translation3d trans = redPose.getTranslation();
    double reflectedX = 2 * midX - trans.getX();
    Translation3d newTrans = new Translation3d(reflectedX, trans.getY(), trans.getZ());

    // Adjust the rotation: keep roll and pitch the same, but set yaw to π - (old yaw).
    double roll = redPose.getRotation().getX();
    double pitch = redPose.getRotation().getY();
    double yaw = redPose.getRotation().getZ();
    double newYaw = Math.PI - yaw;
    Rotation3d newRot = new Rotation3d(roll, pitch, newYaw);

    return new Pose3d(newTrans, newRot);
  }

}
