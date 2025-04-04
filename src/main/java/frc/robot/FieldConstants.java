package frc.robot;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;
import static frc.robot.util.AlignmentUtils.ReefFaceSelection.PolePosition.*;

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
import frc.robot.util.AlignmentUtils.ReefFaceSelection.PolePosition;
import frc.robot.util.NewBranchTranslations;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class FieldConstants {

  public static final double THRESHOLD_DISTANCE_FOR_DRIVE_TO_POLE = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_REEF = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_ROTATION_TO_STATION = 2.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_AUTOMATIC_CAGE_ASSIST = 1.0; // meters
  public static final double THRESHOLD_DISTANCE_FOR_DISLODGE = 2.0; // meters

  public static Translation2d selectedCageTranslation = new Translation2d();

  // Assume these are your fixed cage positions defined somewhere (using WPILib's
  // coordinates)
  private static Translation2d BLUE_CAGE_LEFT = new Translation2d(8.56, 7.26);
  private static Translation2d BLUE_CAGE_MID = new Translation2d(8.56, 6.18);
  private static Translation2d BLUE_CAGE_RIGHT = new Translation2d(8.56, 5.07);

  private static Translation2d RED_CAGE_LEFT = new Translation2d(9.0, 2.98);
  private static Translation2d RED_CAGE_MID = new Translation2d(9.0, 1.87);
  private static Translation2d RED_CAGE_RIGHT = new Translation2d(9.0, 0.78);

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
  public enum
      ReefBranchesBlue { // this is the pose the robot needs to be at to score on these branches
    // BRANCH_A_ID_18_FACE_1_POLE_A(new Translation2d(3.175, 4.360)),
    // BRANCH_B_ID_18_FACE_1_POLE_B(new Translation2d(3.175, 4.067)),
    // BRANCH_C_ID_17_FACE_6_POLE_A(new Translation2d(3.572, 3.036)),
    // BRANCH_D_ID_17_FACE_6_POLE_B(new Translation2d(3.827, 2.895)),
    // BRANCH_E_ID_22_FACE_5_POLE_A(new Translation2d(4.861, 2.714)),
    // BRANCH_F_ID_22_FACE_5_POLE_B(new Translation2d(5.155, 2.906)),
    // BRANCH_G_ID_21_FACE_4_POLE_A(new Translation2d(5.785, 3.694)),
    // BRANCH_H_ID_21_FACE_4_POLE_B(new Translation2d(5.785, 4.040)),
    // BRANCH_I_ID_20_FACE_3_POLE_A(new Translation2d(5.416, 5.001)),
    // BRANCH_J_ID_20_FACE_3_POLE_B(new Translation2d(5.122, 5.168)),
    // BRANCH_K_ID_19_FACE_2_POLE_A(new Translation2d(4.124, 5.330)),
    // BRANCH_L_ID_19_FACE_2_POLE_B(new Translation2d(3.842, 5.159));

    BRANCH_A_ID_18_FACE_1_POLE_A(NewBranchTranslations.getLeftBranchTranslation(18)),
    BRANCH_B_ID_18_FACE_1_POLE_B(NewBranchTranslations.getRightBranchTranslation(18)),
    BRANCH_C_ID_17_FACE_6_POLE_A(NewBranchTranslations.getLeftBranchTranslation(17)),
    BRANCH_D_ID_17_FACE_6_POLE_B(NewBranchTranslations.getRightBranchTranslation(17)),
    BRANCH_E_ID_22_FACE_5_POLE_A(NewBranchTranslations.getLeftBranchTranslation(22)),
    BRANCH_F_ID_22_FACE_5_POLE_B(NewBranchTranslations.getRightBranchTranslation(22)),
    BRANCH_G_ID_21_FACE_4_POLE_A(NewBranchTranslations.getLeftBranchTranslation(21)),
    BRANCH_H_ID_21_FACE_4_POLE_B(NewBranchTranslations.getRightBranchTranslation(21)),
    BRANCH_I_ID_20_FACE_3_POLE_A(NewBranchTranslations.getLeftBranchTranslation(20)),
    BRANCH_J_ID_20_FACE_3_POLE_B(NewBranchTranslations.getRightBranchTranslation(20)),
    BRANCH_K_ID_19_FACE_2_POLE_A(NewBranchTranslations.getLeftBranchTranslation(19)),
    BRANCH_L_ID_19_FACE_2_POLE_B(NewBranchTranslations.getRightBranchTranslation(19));

    private final Translation2d branchTranslation;

    ReefBranchesBlue(Translation2d branchTranslation) {
      this.branchTranslation = branchTranslation;
    }

    public Translation2d getBranchTranslation() {
      return branchTranslation;
    }
  }

  public enum ReefFacesBlue {
    FACE_17(
        17,
        ReefBranchesBlue.BRANCH_C_ID_17_FACE_6_POLE_A,
        ReefBranchesBlue.BRANCH_D_ID_17_FACE_6_POLE_B),
    FACE_18(
        18,
        ReefBranchesBlue.BRANCH_A_ID_18_FACE_1_POLE_A,
        ReefBranchesBlue.BRANCH_B_ID_18_FACE_1_POLE_B),
    FACE_19(
        19,
        ReefBranchesBlue.BRANCH_K_ID_19_FACE_2_POLE_A,
        ReefBranchesBlue.BRANCH_L_ID_19_FACE_2_POLE_B),
    FACE_20(
        20,
        ReefBranchesBlue.BRANCH_I_ID_20_FACE_3_POLE_A,
        ReefBranchesBlue.BRANCH_J_ID_20_FACE_3_POLE_B),
    FACE_21(
        21,
        ReefBranchesBlue.BRANCH_G_ID_21_FACE_4_POLE_A,
        ReefBranchesBlue.BRANCH_H_ID_21_FACE_4_POLE_B),
    FACE_22(
        22,
        ReefBranchesBlue.BRANCH_E_ID_22_FACE_5_POLE_A,
        ReefBranchesBlue.BRANCH_F_ID_22_FACE_5_POLE_B);

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
    BRANCH_A_ID_7_FACE_1_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_A_ID_18_FACE_1_POLE_A.getBranchTranslation())),
    BRANCH_B_ID_7_FACE_1_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_B_ID_18_FACE_1_POLE_B.getBranchTranslation())),
    BRANCH_C_ID_8_FACE_6_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_C_ID_17_FACE_6_POLE_A.getBranchTranslation())),
    BRANCH_D_ID_8_FACE_6_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_D_ID_17_FACE_6_POLE_B.getBranchTranslation())),
    BRANCH_E_ID_9_FACE_5_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_E_ID_22_FACE_5_POLE_A.getBranchTranslation())),
    BRANCH_F_ID_9_FACE_5_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_F_ID_22_FACE_5_POLE_B.getBranchTranslation())),
    BRANCH_G_ID_10_FACE_4_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_G_ID_21_FACE_4_POLE_A.getBranchTranslation())),
    BRANCH_H_ID_10_FACE_4_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_H_ID_21_FACE_4_POLE_B.getBranchTranslation())),
    BRANCH_I_ID_11_FACE_3_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_I_ID_20_FACE_3_POLE_A.getBranchTranslation())),
    BRANCH_J_ID_11_FACE_3_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_J_ID_20_FACE_3_POLE_B.getBranchTranslation())),
    BRANCH_K_ID_6_FACE_2_POLE_A(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_K_ID_19_FACE_2_POLE_A.getBranchTranslation())),
    BRANCH_L_ID_6_FACE_2_POLE_B(
        reflectBlueToRed(ReefBranchesBlue.BRANCH_L_ID_19_FACE_2_POLE_B.getBranchTranslation()));

    private final Translation2d translation;

    ReefBranchesRed(Translation2d translation) {
      this.translation = translation;
    }

    public Translation2d getBranchTranslation() {
      return translation;
    }
  }

  public enum ReefFacesRed {
    FACE_6(
        6,
        ReefBranchesRed.BRANCH_K_ID_6_FACE_2_POLE_A,
        ReefBranchesRed.BRANCH_L_ID_6_FACE_2_POLE_B),
    FACE_7(
        7,
        ReefBranchesRed.BRANCH_A_ID_7_FACE_1_POLE_A,
        ReefBranchesRed.BRANCH_B_ID_7_FACE_1_POLE_B),
    FACE_8(
        8,
        ReefBranchesRed.BRANCH_C_ID_8_FACE_6_POLE_A,
        ReefBranchesRed.BRANCH_D_ID_8_FACE_6_POLE_B),
    FACE_9(
        9,
        ReefBranchesRed.BRANCH_E_ID_9_FACE_5_POLE_A,
        ReefBranchesRed.BRANCH_F_ID_9_FACE_5_POLE_B),
    FACE_10(
        10,
        ReefBranchesRed.BRANCH_G_ID_10_FACE_4_POLE_A,
        ReefBranchesRed.BRANCH_H_ID_10_FACE_4_POLE_B),
    FACE_11(
        11,
        ReefBranchesRed.BRANCH_I_ID_11_FACE_3_POLE_A,
        ReefBranchesRed.BRANCH_J_ID_11_FACE_3_POLE_B);

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
    double fieldLengthY = 8.05; // total field length in X meters

    double xBlue = blueCoord.getX();
    double yBlue = blueCoord.getY();

    double xRed = fieldWidthX - xBlue; // reflect in X
    double yRed = fieldLengthY - yBlue; // Y stays the same

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
      Translation2d left = face.getLeftPole().getBranchTranslation();
      Translation2d right = face.getRightPole().getBranchTranslation();
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
    for (ReefFacesRed face : ReefFacesRed.values()) {
      // Compute the red face ID by subtracting 11.
      int redFaceId = face.getFaceId(); // e.g., 17-11 = 6, 18-11 = 7, etc.

      Translation2d left = face.getLeftPole().getBranchTranslation();
      Translation2d right = face.getRightPole().getBranchTranslation();
      double midX = (left.getX() + right.getX()) / 2.0;
      double midY = (left.getY() + right.getY()) / 2.0;
      Translation2d midpoint = new Translation2d(midX, midY);

      // Retrieve the blue orientation(s) for this face id.
      Rotation2d[] possibleOrients = REEF_FACE_ORIENTATION_RED.get(redFaceId);
      Rotation2d chosenRotation =
          (possibleOrients != null && possibleOrients.length > 0)
              ? possibleOrients[0] // Default to the first orientation.
              : new Rotation2d(); // Fallback to 0 radians.

      redPoses.put(redFaceId, new Pose2d(midpoint, chosenRotation));
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
        return CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1;
      } else if (DISLODGE_L2_FACES_BLUE.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2;
      }
    } else { // Alliance.Red
      if (DISLODGE_L1_FACES_RED.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1;
      } else if (DISLODGE_L2_FACES_RED.contains(faceId)) {
        return CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2;
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
  public static final Translation3d RED_REEF_CENTER_ONSHAPE = new Translation3d(4.280, 0, 0);
  public static final Translation3d BLUE_REEF_CENTER_ONSHAPE = new Translation3d(-4.280, 0, 0);

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

  /** Onshape(origin field center) to WPILib coordinates (origin blue corner). */
  public static Translation3d convertCenteredToWPILib(Translation3d onshapePos) {
    double fieldLength = 17.55;
    double fieldWidth = 8.05;
    double halfLength = fieldLength / 2.0; // ~8.775 m
    double halfWidth = fieldWidth / 2.0; // ~4.025 m (adjust if necessary)

    return new Translation3d(
        onshapePos.getX() + halfLength, onshapePos.getY() + halfWidth, onshapePos.getZ());
  }

  /** Rotates a Translation3d about the Z-axis by the given angle (in radians). */
  public static Translation3d rotateTranslationByAngle(Translation3d translation, double angleRad) {
    double x = translation.getX();
    double y = translation.getY();
    double rotatedX = x * Math.cos(angleRad) - y * Math.sin(angleRad);
    double rotatedY = x * Math.sin(angleRad) + y * Math.cos(angleRad);
    return new Translation3d(rotatedX, rotatedY, translation.getZ());
  }

  public static class CoralKey {
    public final CoralSystemPresets preset; // e.g. "L1", "L2", etc.
    public final int faceId; // aprilTag Id of Face
    public final PolePosition pole; // "A" or "B"

    public CoralKey(int faceId, PolePosition pole, CoralSystemPresets preset) {
      this.faceId = faceId;
      this.pole = pole;
      this.preset = preset;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof CoralKey)) return false;
      CoralKey key = (CoralKey) o;
      return faceId == key.faceId && preset.equals(key.preset) && pole.equals(key.pole);
    }

    @Override
    public int hashCode() {
      return Objects.hash(faceId, pole, preset);
    }

    @Override
    public String toString() {
      return faceId + "_" + pole + "_" + preset;
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
            RED_REEF_CENTER_ONSHAPE,
            L1_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L1_HEIGHT,
            L1_FIXED_PITCH_DEG,
            L1_BASE_ANGLE_DEG,
            L1_YAW_OFFSET_DEG);
    int[] redFaceIdsForL1 = {7, 8, 9, 10, 11, 6}; // example order for each face
    populateCoralMappingForPreset(redL1, CoralSystemPresets.L1_SCORE, redFaceIdsForL1);

    Pose3d[] redL2 =
        generateScoredCoralPosesAngled(
            RED_REEF_CENTER_ONSHAPE,
            L2_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L2_HEIGHT,
            L2_FIXED_PITCH_DEG,
            L2_BASE_ANGLE_DEG);
    int[] redFaceIdsForL2 = {7, 8, 9, 10, 11, 6}; // example order for each face
    populateCoralMappingForPreset(redL2, CoralSystemPresets.L2, redFaceIdsForL2);

    Pose3d[] redL3 =
        generateScoredCoralPosesAngled(
            RED_REEF_CENTER_ONSHAPE,
            L3_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L3_HEIGHT,
            L3_FIXED_PITCH_DEG,
            L3_BASE_ANGLE_DEG);
    int[] redFaceIdsForL3 = {7, 8, 9, 10, 11, 6}; // example order for each face
    populateCoralMappingForPreset(redL3, CoralSystemPresets.L3, redFaceIdsForL3);

    Pose3d[] redL4 =
        generateScoredCoralPosesL4(
            RED_REEF_CENTER_ONSHAPE,
            L4_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L4_HEIGHT,
            L4_FIXED_PITCH_DEG,
            L4_BASE_ANGLE_DEG);
    int[] redFaceIdsForL4 = {7, 8, 9, 10, 11, 6}; // example order for each face
    populateCoralMappingForPreset(redL4, CoralSystemPresets.L4, redFaceIdsForL4);

    // Build red coral mapping for L1
    Pose3d[] blueL1 =
        generateScoredCoralPosesL1(
            BLUE_REEF_CENTER_ONSHAPE,
            L1_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L1_HEIGHT,
            L1_FIXED_PITCH_DEG,
            L1_BASE_ANGLE_DEG,
            L1_YAW_OFFSET_DEG);
    coralMapping.put(new CoralKey(21, A_LEFT, L1_SCORE), new CoralLocation(blueL1[0]));
    coralMapping.put(new CoralKey(21, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[1]));
    coralMapping.put(new CoralKey(22, A_LEFT, L1_SCORE), new CoralLocation(blueL1[10]));
    coralMapping.put(new CoralKey(22, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[11]));
    coralMapping.put(new CoralKey(17, A_LEFT, L1_SCORE), new CoralLocation(blueL1[8]));
    coralMapping.put(new CoralKey(17, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[9]));
    coralMapping.put(new CoralKey(18, A_LEFT, L1_SCORE), new CoralLocation(blueL1[6]));
    coralMapping.put(new CoralKey(18, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[7]));
    coralMapping.put(new CoralKey(19, A_LEFT, L1_SCORE), new CoralLocation(blueL1[4]));
    coralMapping.put(new CoralKey(19, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[5]));
    coralMapping.put(new CoralKey(20, A_LEFT, L1_SCORE), new CoralLocation(blueL1[2]));
    coralMapping.put(new CoralKey(20, B_RIGHT, L1_SCORE), new CoralLocation(blueL1[3]));

    Pose3d[] blueL2 =
        generateScoredCoralPosesAngled(
            BLUE_REEF_CENTER_ONSHAPE,
            L2_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L2_HEIGHT,
            L2_FIXED_PITCH_DEG,
            L2_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey(21, A_LEFT, L2), new CoralLocation(blueL2[0]));
    coralMapping.put(new CoralKey(21, B_RIGHT, L2), new CoralLocation(blueL2[1]));
    coralMapping.put(new CoralKey(22, A_LEFT, L2), new CoralLocation(blueL2[10]));
    coralMapping.put(new CoralKey(22, B_RIGHT, L2), new CoralLocation(blueL2[11]));
    coralMapping.put(new CoralKey(17, A_LEFT, L2), new CoralLocation(blueL2[8]));
    coralMapping.put(new CoralKey(17, B_RIGHT, L2), new CoralLocation(blueL2[9]));
    coralMapping.put(new CoralKey(18, A_LEFT, L2), new CoralLocation(blueL2[6]));
    coralMapping.put(new CoralKey(18, B_RIGHT, L2), new CoralLocation(blueL2[7]));
    coralMapping.put(new CoralKey(19, A_LEFT, L2), new CoralLocation(blueL2[4]));
    coralMapping.put(new CoralKey(19, B_RIGHT, L2), new CoralLocation(blueL2[5]));
    coralMapping.put(new CoralKey(20, A_LEFT, L2), new CoralLocation(blueL2[2]));
    coralMapping.put(new CoralKey(20, B_RIGHT, L2), new CoralLocation(blueL2[3]));

    Pose3d[] blueL3 =
        generateScoredCoralPosesAngled(
            BLUE_REEF_CENTER_ONSHAPE,
            L3_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L3_HEIGHT,
            L3_FIXED_PITCH_DEG,
            L3_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey(21, A_LEFT, L3), new CoralLocation(blueL3[0]));
    coralMapping.put(new CoralKey(21, B_RIGHT, L3), new CoralLocation(blueL3[1]));
    coralMapping.put(new CoralKey(22, A_LEFT, L3), new CoralLocation(blueL3[10]));
    coralMapping.put(new CoralKey(22, B_RIGHT, L3), new CoralLocation(blueL3[11]));
    coralMapping.put(new CoralKey(17, A_LEFT, L3), new CoralLocation(blueL3[8]));
    coralMapping.put(new CoralKey(17, B_RIGHT, L3), new CoralLocation(blueL3[9]));
    coralMapping.put(new CoralKey(18, A_LEFT, L3), new CoralLocation(blueL3[6]));
    coralMapping.put(new CoralKey(18, B_RIGHT, L3), new CoralLocation(blueL3[7]));
    coralMapping.put(new CoralKey(19, A_LEFT, L3), new CoralLocation(blueL3[4]));
    coralMapping.put(new CoralKey(19, B_RIGHT, L3), new CoralLocation(blueL3[5]));
    coralMapping.put(new CoralKey(20, A_LEFT, L3), new CoralLocation(blueL3[2]));
    coralMapping.put(new CoralKey(20, B_RIGHT, L3), new CoralLocation(blueL3[3]));

    Pose3d[] blueL4 =
        generateScoredCoralPosesL4(
            BLUE_REEF_CENTER_ONSHAPE,
            L4_EFFECTIVE_RADIUS,
            POLE_OFFSET,
            L4_HEIGHT,
            L4_FIXED_PITCH_DEG,
            L4_BASE_ANGLE_DEG);
    coralMapping.put(new CoralKey(21, A_LEFT, L4), new CoralLocation(blueL4[0]));
    coralMapping.put(new CoralKey(21, B_RIGHT, L4), new CoralLocation(blueL4[1]));
    coralMapping.put(new CoralKey(22, A_LEFT, L4), new CoralLocation(blueL4[10]));
    coralMapping.put(new CoralKey(22, B_RIGHT, L4), new CoralLocation(blueL4[11]));
    coralMapping.put(new CoralKey(17, A_LEFT, L4), new CoralLocation(blueL4[8]));
    coralMapping.put(new CoralKey(17, B_RIGHT, L4), new CoralLocation(blueL4[9]));
    coralMapping.put(new CoralKey(18, A_LEFT, L4), new CoralLocation(blueL4[6]));
    coralMapping.put(new CoralKey(18, B_RIGHT, L4), new CoralLocation(blueL4[7]));
    coralMapping.put(new CoralKey(19, A_LEFT, L4), new CoralLocation(blueL4[4]));
    coralMapping.put(new CoralKey(19, B_RIGHT, L4), new CoralLocation(blueL4[5]));
    coralMapping.put(new CoralKey(20, A_LEFT, L4), new CoralLocation(blueL4[2]));
    coralMapping.put(new CoralKey(20, B_RIGHT, L4), new CoralLocation(blueL4[3]));
  }

  private static void populateCoralMappingForPreset(
      Pose3d[] coralPoses, CoralSystemPresets preset, int[] redFaceIds) {

    // We assume coralPoses.length == 12 and redFaceIds.length == 6.
    for (int i = 0; i < 6; i++) {
      int faceId = redFaceIds[i];
      // Each face provides two poles (assumed to be consecutive in the array)
      Pose3d leftPose = coralPoses[2 * i]; // Left pole, guaranteed by upstream order.
      Pose3d rightPose = coralPoses[2 * i + 1]; // Right pole

      // Add the entries to coralMapping using your CoralKey keys.
      coralMapping.put(new CoralKey(faceId, A_LEFT, preset), new CoralLocation(leftPose));
      coralMapping.put(new CoralKey(faceId, B_RIGHT, preset), new CoralLocation(rightPose));
    }
  }

  public static final Map<Integer, Translation2d> BLUE_PROCESSOR_APRIL_TAG =
      new HashMap<>(); // Blue
  public static final Map<Integer, Translation2d> RED_PROCESSOR_APRIL_TAG = new HashMap<>(); // Red

  static {
    // ---------------- BLUE PROCESSOR TAG ----------------
    BLUE_PROCESSOR_APRIL_TAG.put(16, new Translation2d(6.03, .46));
    // ---------------- RED PROCESSOR TAG -----
    RED_PROCESSOR_APRIL_TAG.put(3, new Translation2d(11.63, 7.61));
  }

  /** Orientation data for BLUE reef faces, keyed by faceId 12-13 */
  public static final Rotation2d PROCESSOR_ORIENTATION_BLUE = new Rotation2d(Math.toRadians(180));

  /** Orientation data for RED reef faces, keyed by faceId 1-2 */
  public static final Rotation2d PROCESSOR_ORIENTATION_RED = new Rotation2d(Math.toRadians(0));
}
