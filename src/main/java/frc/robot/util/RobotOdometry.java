package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class RobotOdometry {
  private static Drive driveSubsystem;

  // Called once from RobotContainer to initialize with the actual Drive instance
  public static void initialize(Drive drive) {
    driveSubsystem = drive;
  }

  // Globally accessible method to get the robot's pose
  public static Pose2d getRobotPose() {
    if (driveSubsystem == null) {
      throw new IllegalStateException("RobotOdometry has not been initialized!");
    }
    return driveSubsystem.getPose();
  }
}
