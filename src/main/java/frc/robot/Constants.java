// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Map<Integer, Translation2d> BLUE_APRIL_TAGS = new HashMap<>(); // Blue
  public static final Map<Integer, Translation2d> RED_APRIL_TAGS = new HashMap<>(); // Red

  static {
    // ---------------- BLUE REEF APRIL TAGS ----------------
    BLUE_APRIL_TAGS.put(17, new Translation2d(4.073905999999999, 3.3063179999999996));
    BLUE_APRIL_TAGS.put(18, new Translation2d(3.6576, 4.0259));
    BLUE_APRIL_TAGS.put(19, new Translation2d(4.073905999999999, 4.745482));
    BLUE_APRIL_TAGS.put(20, new Translation2d(4.904739999999999, 4.745482));
    BLUE_APRIL_TAGS.put(21, new Translation2d(5.321046, 4.0259));
    BLUE_APRIL_TAGS.put(22, new Translation2d(4.904739999999999, 3.3063179999999996));

    // ---------------- RED REEF APRIL TAGS ----------------
    RED_APRIL_TAGS.put(6, new Translation2d(13.474446, 3.3063179999999996));
    RED_APRIL_TAGS.put(7, new Translation2d(13.890498, 4.0259));
    RED_APRIL_TAGS.put(8, new Translation2d(13.47444, 4.745482));
    RED_APRIL_TAGS.put(9, new Translation2d(12.643358, 4.745482));
    RED_APRIL_TAGS.put(10, new Translation2d(12.227305999999999, 4.0259));
    RED_APRIL_TAGS.put(11, new Translation2d(12.643358, 3.3063179999999996));
  }

  public enum ReefOrientationType {
    FRONT,
    BACK
  }

  /**
   * A result that includes both the chosen angle and which orientation (front/back) it came from.
   */
  public record ChosenOrientation(Rotation2d rotation2D, ReefOrientationType orientationType) {}

  /** Orientation data for BLUE reef faces, keyed by faceId 17–22. */
  public static final Map<Integer, Rotation2d[]> REEF_FACE_ORIENTATION_BLUE = new HashMap<>();

  /** Orientation data for RED reef faces, keyed by faceId 6–11. */
  public static final Map<Integer, Rotation2d[]> REEF_FACE_ORIENTATION_RED = new HashMap<>();

  static {
    // ---------------- BLUE Orientation ----------------
    REEF_FACE_ORIENTATION_BLUE.put(
        17,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-30)), new Rotation2d(Math.toRadians(150))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        18,
        new Rotation2d[] {new Rotation2d(Math.toRadians(-90)), new Rotation2d(Math.toRadians(90))});
    REEF_FACE_ORIENTATION_BLUE.put(
        19,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-150)), new Rotation2d(Math.toRadians(30))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        20,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(150)), new Rotation2d(Math.toRadians(-30))
        });
    REEF_FACE_ORIENTATION_BLUE.put(
        21,
        new Rotation2d[] {new Rotation2d(Math.toRadians(90)), new Rotation2d(Math.toRadians(-90))});
    REEF_FACE_ORIENTATION_BLUE.put(
        22,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(30)), new Rotation2d(Math.toRadians(-150))
        });
    Collections.unmodifiableMap(REEF_FACE_ORIENTATION_BLUE);

    // ---------------- RED Orientation ----------------
    REEF_FACE_ORIENTATION_RED.put(
        6,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(30)), new Rotation2d(Math.toRadians(-150))
        });
    REEF_FACE_ORIENTATION_RED.put(
        7,
        new Rotation2d[] {new Rotation2d(Math.toRadians(90)), new Rotation2d(Math.toRadians(-90))});
    REEF_FACE_ORIENTATION_RED.put(
        8,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(150)), new Rotation2d(Math.toRadians(-30))
        });
    REEF_FACE_ORIENTATION_RED.put(
        9,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-150)), new Rotation2d(Math.toRadians(30))
        });
    REEF_FACE_ORIENTATION_RED.put(
        10,
        new Rotation2d[] {new Rotation2d(Math.toRadians(90)), new Rotation2d(Math.toRadians(-90))});
    REEF_FACE_ORIENTATION_RED.put(
        11,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(-30)), new Rotation2d(Math.toRadians(150))
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
    BRANCH_A(ReefBranchesBlue.BRANCH_A),
    BRANCH_B(ReefBranchesBlue.BRANCH_B),
    BRANCH_C(ReefBranchesBlue.BRANCH_C),
    BRANCH_D(ReefBranchesBlue.BRANCH_D),
    BRANCH_E(ReefBranchesBlue.BRANCH_L),
    BRANCH_F(ReefBranchesBlue.BRANCH_K),
    BRANCH_G(ReefBranchesBlue.BRANCH_G),
    BRANCH_H(ReefBranchesBlue.BRANCH_H),
    BRANCH_I(ReefBranchesBlue.BRANCH_I),
    BRANCH_J(ReefBranchesBlue.BRANCH_J),
    BRANCH_K(ReefBranchesBlue.BRANCH_K),
    BRANCH_L(ReefBranchesBlue.BRANCH_L);

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
    FACE_6(
        6,
        ReefBranchesRed.BRANCH_K,
        ReefBranchesRed.BRANCH_L), // CHECK The placement of the branches (L or R)
    FACE_7(
        7,
        ReefBranchesRed.BRANCH_A,
        ReefBranchesRed.BRANCH_B), // CHECK The placement of the branches (L or R)
    FACE_8(
        8,
        ReefBranchesRed.BRANCH_C,
        ReefBranchesRed.BRANCH_D), // CHECK The placement of the branches (L or R)
    FACE_9(
        9,
        ReefBranchesRed.BRANCH_E,
        ReefBranchesRed.BRANCH_F), // CHECK The placement of the branches (L or R)
    FACE_10(
        10,
        ReefBranchesRed.BRANCH_G,
        ReefBranchesRed.BRANCH_H), // CHECK The placement of the branches (L or R)
    FACE_11(
        11,
        ReefBranchesRed.BRANCH_I,
        ReefBranchesRed.BRANCH_J); // CHECK The placement of the branches (L or R)

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
}
