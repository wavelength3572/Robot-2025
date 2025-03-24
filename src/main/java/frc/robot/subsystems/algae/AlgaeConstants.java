package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 33;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 40;
  public static final int algaeCaptureCurrentLimit = 30;

  public static final double captureIntakeVolts = 10.0;
  public static final double capturePushVolts = -8.0;

  public static final double deployBurstVolts = 1.2; // Inital burst to deploy arm
  public static final double deployHoldOutVolts = 0.5; // Holding out arm waiting for algae capture
  public static final double deployPushAlgaeVolts = 0.6; // Pushing algae out of the arm
  public static final double deployPullBackVolts = -1.2; // Initial burst to stow arm
  public static final double deployHoldVolts =
      -0.4; // Voltage to hold arm back to robot during stow
  public static final double deployPullBackFF = 0.5;

  public static final double kAlgaeCaptureKp = 0.1;
  public static final double kAlgaeCaptureKd = 0.0;

  public static final double kAlgaeDeployKp = 1.5;
  public static final double kAlgaeDeployKd = 0.0;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployInitalAngle = 45.5; // Degrees
  public static final double algaeStowPosition = 1.638; // Encoder Rotations
  public static final double MIN_ANGLE = 45.429;
  public static final double MAX_ANGLE = 135.0;

  public static enum algaeIntakeState {
    OFF,
    BURST,
    PULL,
    DETECT,
    CAPTURE,
    PULL_ARM,
    PUSH,
    STOW,
    CLIMB,
    MANUAL
  }
}
