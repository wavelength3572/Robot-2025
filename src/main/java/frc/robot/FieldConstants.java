package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

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
          new Rotation2d(Math.toRadians(60)), new Rotation2d(Math.toRadians(-120)) // 60, -120
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        18,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(0)), new Rotation2d(Math.toRadians(180))
        }); // 180, -180
    REEF_FACE_ORIENTATION_BLUE.put(
        19,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-60)), new Rotation2d(Math.toRadians(120)) // 120, -60
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        20,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-120)), new Rotation2d(Math.toRadians(60))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        21,
        new Rotation2d[] {new Rotation2d(Math.toRadians(180)), new Rotation2d(Math.toRadians(0))});
    REEF_FACE_ORIENTATION_BLUE.put(
        22,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(120)), new Rotation2d(Math.toRadians(-60))
        });
    Collections.unmodifiableMap(REEF_FACE_ORIENTATION_BLUE);

    // ---------------- RED Orientation ----------------
    REEF_FACE_ORIENTATION_RED.put(
        6,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-60)), new Rotation2d(Math.toRadians(120))
        });
    REEF_FACE_ORIENTATION_RED.put(
        7,
        new Rotation2d[] {new Rotation2d(Math.toRadians(0)), new Rotation2d(Math.toRadians(180))});
    REEF_FACE_ORIENTATION_RED.put(
        8,
        new Rotation2d[] {new Rotation2d(Math.toRadians(60)), new Rotation2d(Math.toRadians(240))});
    REEF_FACE_ORIENTATION_RED.put(
        9,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(120)), new Rotation2d(Math.toRadians(-60))
        });
    REEF_FACE_ORIENTATION_RED.put(
        10,
        new Rotation2d[] {new Rotation2d(Math.toRadians(180)), new Rotation2d(Math.toRadians(0))});
    REEF_FACE_ORIENTATION_RED.put(
        11,
        new Rotation2d[] {new Rotation2d(Math.toRadians(240)), new Rotation2d(Math.toRadians(60))});
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
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(54)), new Rotation2d(Math.toRadians(54 + 180))
        });
    CORAL_STATION_ORIENTATION_BLUE.put(
        13,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(306)), new Rotation2d(Math.toRadians(306 + 180))
        }); // 180, -180
    Collections.unmodifiableMap(CORAL_STATION_ORIENTATION_BLUE);

    // ---------------- RED Orientation ----------------
    CORAL_STATION_ORIENTATION_RED.put(
        1,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(126)), new Rotation2d(Math.toRadians(126 + 180))
        });
    CORAL_STATION_ORIENTATION_RED.put(
        2,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(234)), new Rotation2d(Math.toRadians(234 + 180))
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
}
