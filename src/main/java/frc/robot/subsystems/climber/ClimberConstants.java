package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ClimberConstants {
  public static final int canId = 40;
  public static final int climberCurrentLimit = 50;
  public static final double deployPower = -1.0;
  public static final double climbPower = 1.0;

  // Define the climber tip offset relative to the robot's center
  public static final double CLIMBER_TIP_OFFSET_X = 0.35;
  public static final double CLIMBER_TIP_OFFSET_Y = 0.06;

  /**
   * Computes the climber tip pose based on the robot's current pose.
   *
   * @param robotPose The current pose of the robot.
   * @return The pose of the climber tip.
   */
  public static Pose2d computeClimberTipPose(Pose2d robotPose) {
    return robotPose.transformBy(
        new Transform2d(
            new Translation2d(CLIMBER_TIP_OFFSET_X, CLIMBER_TIP_OFFSET_Y),
            robotPose.getRotation()));
  }

  public static enum CLIMB_STATE {
    DEPLOY,
    STOWED,
    CLIMB,
    FINAL
  }
}
