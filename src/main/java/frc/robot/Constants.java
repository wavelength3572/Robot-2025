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

  // Example: 6 reef-face positions as a static array
  public static final Map<Integer, Translation2d> REEF_FACES = new HashMap<>();

  static {
    REEF_FACES.put(17, new Translation2d(4.073905999999999, 3.3063179999999996));
    REEF_FACES.put(18, new Translation2d(3.6576, 4.0259));
    REEF_FACES.put(19, new Translation2d(4.073905999999999, 4.745482));
    REEF_FACES.put(20, new Translation2d(4.904739999999999, 4.745482));
    REEF_FACES.put(21, new Translation2d(5.321046, 4.0259));
    REEF_FACES.put(22, new Translation2d(4.904739999999999, 3.3063179999999996));
  }
  ;

  public static final Map<Integer, Rotation2d[]> REEF_FACE_ORIENTATION = new HashMap<>();

  static {
    REEF_FACE_ORIENTATION.put(
        17,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(150)), new Rotation2d(Math.toRadians(-30))
        });
    REEF_FACE_ORIENTATION.put(
        18,
        new Rotation2d[] {new Rotation2d(Math.toRadians(90)), new Rotation2d(Math.toRadians(-90))});
    REEF_FACE_ORIENTATION.put(
        19,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(30)), new Rotation2d(Math.toRadians(-150))
        });
    REEF_FACE_ORIENTATION.put(
        20,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(150)), new Rotation2d(Math.toRadians(-30))
        });
    REEF_FACE_ORIENTATION.put(
        21,
        new Rotation2d[] {new Rotation2d(Math.toRadians(90)), new Rotation2d(Math.toRadians(-90))});
    REEF_FACE_ORIENTATION.put(
        22,
        new Rotation2d[] {
          new Rotation2d(Math.toRadians(30)), new Rotation2d(Math.toRadians(-150))
        });
    Collections.unmodifiableMap(REEF_FACE_ORIENTATION);
  }

  public enum ReefBranches {
    BRANCH_A(new Translation2d(3.2385, 4.3807)),
    BRANCH_B(new Translation2d(3.2385, 4.0521)),
    BRANCH_C(new Translation2d(3.5566, 3.1201)),
    BRANCH_D(new Translation2d(3.7863, 2.9875)),
    BRANCH_E(new Translation2d(4.8075, 2.7652)),
    BRANCH_F(new Translation2d(5.0921, 2.9296)),
    BRANCH_G(new Translation2d(5.7402, 3.6711)),
    BRANCH_H(new Translation2d(5.7402, 3.9997)),
    BRANCH_I(new Translation2d(5.4220, 4.9317)),
    BRANCH_J(new Translation2d(5.1374, 5.0961)),
    BRANCH_K(new Translation2d(4.1712, 5.2866)),
    BRANCH_L(new Translation2d(3.8866, 5.1222));

    private final Translation2d translation;

    ReefBranches(Translation2d translation) {
      this.translation = translation;
    }

    public Translation2d getTranslation() {
      return translation;
    }
  }

  public enum ReefFaces {
    FACE_17(17, ReefBranches.BRANCH_C, ReefBranches.BRANCH_D),
    FACE_18(18, ReefBranches.BRANCH_A, ReefBranches.BRANCH_B),
    FACE_19(19, ReefBranches.BRANCH_K, ReefBranches.BRANCH_L),
    FACE_20(20, ReefBranches.BRANCH_I, ReefBranches.BRANCH_J),
    FACE_21(21, ReefBranches.BRANCH_G, ReefBranches.BRANCH_H),
    FACE_22(22, ReefBranches.BRANCH_E, ReefBranches.BRANCH_F);

    private final int faceId;
    private final ReefBranches leftPole;
    private final ReefBranches rightPole;

    ReefFaces(int faceId, ReefBranches leftPole, ReefBranches rightPole) {
      this.faceId = faceId;
      this.leftPole = leftPole;
      this.rightPole = rightPole;
    }

    public int getFaceId() {
      return faceId;
    }

    public ReefBranches getLeftPole() {
      return leftPole;
    }

    public ReefBranches getRightPole() {
      return rightPole;
    }

    /**
     * Find the ReefFaces enum constant matching the given faceId, or return null if none matches.
     */
    public static ReefFaces fromId(int faceId) {
      for (ReefFaces face : values()) {
        if (face.faceId == faceId) {
          return face;
        }
      }
      return null; // or throw an IllegalArgumentException
    }
  }
}
