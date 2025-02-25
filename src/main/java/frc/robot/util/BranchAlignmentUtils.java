package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.ReefFacesBlue;
import frc.robot.FieldConstants.ReefFacesRed;
import org.littletonrobotics.junction.Logger;

public final class BranchAlignmentUtils {

  public enum BranchAlignmentStatus {
    NONE,
    RED,
    YELLOW,
    GREEN
  }

  // ----------------- TUNE THESE DISTANCES (meters) -----------------
  private static final double THRESHOLD_RED = 0.15; // If distance < 1.0m => at least RED
  private static final double THRESHOLD_YELLOW = 0.05; // If distance < 0.5m => at least YELLOW
  private static final double THRESHOLD_GREEN = 0.01; // If distance < 0.25m => GREEN

  private BranchAlignmentUtils() {
    // prevent instantiation
  }

  /**
   * Returns how close the robot is to the nearest branch (pole) on a given reef face, using only
   * the front translation of each pole. The result is a traffic-light style status: NONE, RED,
   * YELLOW, GREEN.
   *
   * @param robotPose The robot's current position.
   * @param faceId The reef face’s AprilTag ID (e.g., 17..22 for Blue or 6..11 for Red).
   * @return BranchAlignmentStatus.
   */
  public static BranchAlignmentStatus getBranchAlignmentStatus(Pose2d robotPose, int faceId) {
    // 1) Which alliance are we?
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    Logger.recordOutput("Alignment/Branch/Alliance", alliance.toString());
    Logger.recordOutput("Alignment/Branch/FaceId", faceId);

    double minDistance = Double.POSITIVE_INFINITY; // track the closest branch distance

    // 2) For Blue alliance => look up ReefFacesBlue
    if (alliance == DriverStation.Alliance.Blue) {
      ReefFacesBlue faceEnum = ReefFacesBlue.fromId(faceId);
      if (faceEnum != null) {
        // Use only the front translation from left and right poles.
        Translation2d leftFront = faceEnum.getLeftPole().getBranchTranslation();
        Translation2d rightFront = faceEnum.getRightPole().getBranchTranslation();

        double distLF = robotPose.getTranslation().getDistance(leftFront);
        double distRF = robotPose.getTranslation().getDistance(rightFront);

        Logger.recordOutput("Alignment/Branch/Blue/LeftFront", leftFront);
        Logger.recordOutput("Alignment/Branch/Blue/RightFront", rightFront);
        Logger.recordOutput("Alignment/Branch/Blue/DistanceLeftFront", distLF);
        Logger.recordOutput("Alignment/Branch/Blue/DistanceRightFront", distRF);

        minDistance = Math.min(distLF, distRF);
      }
    }
    // 3) For Red alliance => look up ReefFacesRed
    else if (alliance == DriverStation.Alliance.Red) {
      ReefFacesRed faceEnum = ReefFacesRed.fromId(faceId);
      if (faceEnum != null) {
        Translation2d leftFront = faceEnum.getLeftPole().getBranchTranslation();
        Translation2d rightFront = faceEnum.getRightPole().getBranchTranslation();

        double distLF = robotPose.getTranslation().getDistance(leftFront);
        double distRF = robotPose.getTranslation().getDistance(rightFront);

        Logger.recordOutput("Alignment/Branch/Red/LeftFront", leftFront);
        Logger.recordOutput("Alignment/Branch/Red/RightFront", rightFront);
        Logger.recordOutput("Alignment/Branch/Red/DistanceLeftFront", distLF);
        Logger.recordOutput("Alignment/Branch/Red/DistanceRightFront", distRF);

        minDistance = Math.min(distLF, distRF);
      }
    }
    // If alliance is invalid or no face was found, minDistance remains Infinity

    Logger.recordOutput("Alignment/Branch/MinDistance", minDistance);

    // 4) Compare minDistance to thresholds to decide the traffic-light status
    BranchAlignmentStatus status;
    if (minDistance == Double.POSITIVE_INFINITY) {
      status = BranchAlignmentStatus.NONE;
    } else if (minDistance > THRESHOLD_RED) {
      status = BranchAlignmentStatus.NONE;
    } else if (minDistance > THRESHOLD_YELLOW) {
      status = BranchAlignmentStatus.RED;
    } else if (minDistance > THRESHOLD_GREEN) {
      status = BranchAlignmentStatus.YELLOW;
    } else {
      status = BranchAlignmentStatus.GREEN;
    }

    Logger.recordOutput("Alignment/Branch/Status", status.toString());
    return status;
  }
}
