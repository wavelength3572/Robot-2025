package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ClimberConstants {
  public static final int canId = 40;
  public static final int climberCurrentLimit = 50;
  public static final double deployPower = -1.0;
  public static final double climbPower = 1.0;

  // 60:1 Encoder Positions
  // public static final double DEPLOY_POSITION = -500;
  // public static final double FAST_DEPLOY_POSITION = -400;
  // public static final double CLIMBED_POSITION = -140;

  // 25:1 Encoder Positions
  // public static final double DEPLOY_POSITION = -208.3333333333;
  // public static final double FAST_DEPLOY_POSITION = -166.666666666;
  // public static final double CLIMBED_POSITION = -58.33333333333;

  // 20:1 Encoder Positions
  public static final double DEPLOY_POSITION = -155.0;
  public static final double FAST_DEPLOY_POSITION = -133.333333333;
  public static final double CLIMBED_POSITION = -52.4285; // -46.666666666;
  public static final double CLIMBED_SERVO_RELEASE_POSITION = -80; 

  // public static final double DEPLOY_POSITION = -8.33333333; // Drum Position
  // public static final double FAST_DEPLOY_POSITION = -6.6666666; // Drum Position
  // public static final double CLIMBED_POSITION = -2.33333333; // Drum Position

  // Define the climber tip offset relative to the robot's center
  public static final double CLIMBER_TIP_OFFSET_X = 0.35;
  public static final double CLIMBER_TIP_OFFSET_Y = 0.06;

  public static final double climberKp = 0.04;
  public static final double climberKd = 0.0;

  public static final double CLIMBING_TOLERANCE = 4.0;

  public static final double kClimberGearing = 60.0;

  public static final double climberMaxDeploySpeed = -0.8;
  public static final double climberMaxClimbSpeed = 1.0;

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
    STOWED,
    SERVO,
    FAST_DEPLOY,
    DEPLOY,
    CLIMB,
    FINAL
  }
}
