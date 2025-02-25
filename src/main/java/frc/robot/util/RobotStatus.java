package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignmentUtils.CoralStationSelection;
import frc.robot.util.AlignmentUtils.ReefFaceSelection;

public class RobotStatus {
  private static Drive driveSystem;
  private static CoralSystem coralSystem;
  private static Vision visionSystem;
  private static Climber climberSystem;

  // Called once from RobotContainer to initialize with the actual Drive instance
  public static void initialize(Drive drive, CoralSystem coral, Vision vision, Climber climber) {
    driveSystem = drive;
    coralSystem = coral;
    visionSystem = vision;
    climberSystem = climber;
  }

  // Globally accessible method to get the robot's pose
  public static Pose2d getRobotPose() {
    if (driveSystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return driveSystem.getPose();
  }

  // Globally accessible method to get the robot's pose
  public static CoralSystemPresets getCurrentPreset() {
    if (coralSystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return coralSystem.getCurrentCoralPreset();
  }

  // Globally accessible method to get the reefFaceSelection
  public static ReefFaceSelection getReefFaceSelection() {
    if (driveSystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return driveSystem.getReefFaceSelection();
  }

  // Globally accessible method to get the reefFaceSelection
  public static CoralStationSelection getCoralStationSelection() {
    if (driveSystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return driveSystem.getCoralStationSelection();
  }

  // Globally accessible method to get the reefFaceSelection
  public static boolean isDriveModeSmart() {
    return driveSystem.isDriveModeSmart();
  }

  // Globally accessible method to whether vision is on
  public static boolean isVisionOn() {
    return visionSystem.isVisionOn();
  }

  // Globally accessible method to whether vision is on
  public static double elevatorHeightInches() {
    return coralSystem.getElevator().getHeightInInches();
  }

  public static boolean haveCoral() {
    return coralSystem.getIntake().getCoralInRobot();
  }

  public static boolean isClimbingFinished() {
    return climberSystem.isClimbingFinished();
  }
}
