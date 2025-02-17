package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.subsystems.drive.Drive;

public class RobotStatus {
  private static Drive driveSubsystem;
  private static CoralSystem coralSystem;

  // Called once from RobotContainer to initialize with the actual Drive instance
  public static void initialize(Drive drive, CoralSystem coral) {
    driveSubsystem = drive;
    coralSystem = coral;
  }

  // Globally accessible method to get the robot's pose
  public static Pose2d getRobotPose() {
    if (driveSubsystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return driveSubsystem.getPose();
  }

  // Globally accessible method to get the robot's pose
  public static CoralSystemPresets getCurrentPreset() {
    if (coralSystem == null) {
      throw new IllegalStateException("RobotStatus has not been initialized!");
    }
    return coralSystem.getCurrentCoralPreset();
  }
}
