package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.coral.arm.ArmConstants;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import frc.robot.subsystems.coral.endeffector.EndEffectorConstants;

public final class CoralSubsystemConstants {

  public static final double SAFE_CARRIAGE_POSITION = .7;

  // Visualizer dimensions
  public static final double VISUALIZER_WIDTH = 0.8382;
  public static final double VISUALIZER_HEIGHT = 2.0;

  // Root configuration
  public static final String ROOT_NAME = "Base";
  public static final double ROOT_X_POSITION = VISUALIZER_WIDTH / 2.0;
  public static final double ROOT_Y_POSITION = 0.0;

  // Elevator visualization parameters
  public static final double ELEVATOR_INITIAL_LENGTH = ElevatorConstants.kGroundToElevator;
  public static final double ELEVATOR_ANGLE_DEGREES = 90.0;
  public static final double ELEVATOR_THICKNESS = 2.0;
  public static final Color8Bit ELEVATOR_COLOR = new Color8Bit(Color.kBlue);

  // Arm visualization parameters
  public static final double ARM_LENGTH = ArmConstants.kArmLengthMeters;
  public static final double ARM_INITIAL_ANGLE_DEGREES = 0;
  public static final double ARM_THICKNESS = 3.0;
  public static final Color8Bit ARM_COLOR = new Color8Bit(Color.kRed);

  // EndEffector visualization parameters
  public static final double ENDEFFECTOR_LENGTH = EndEffectorConstants.kEndEffectorLengthMeters;
  public static final double ENDEFFECTOR_INITIAL_ANGLE_DEGREES = -90.0;
  public static final double ENDEFFECTOR_THICKNESS = 5.0;
  public static final Color8Bit ENDEFFECTOR_COLOR = new Color8Bit(Color.kBlanchedAlmond);

  private CoralSubsystemConstants() {} // Prevent instantiation
}
