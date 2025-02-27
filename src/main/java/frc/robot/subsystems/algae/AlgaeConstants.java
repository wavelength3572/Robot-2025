package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 33;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 40;
  public static final int algaeCaptureCurrentLimit = 30;

  public static final double captureIntakeVolts = 8.0;
  public static final double capturePushVolts = -8.0;

  public static final double deployBurstVolts = 1.0;
  public static final double deployHoldOutVolts = 0.4;
  public static final double deployPullBackVolts = -1.0;
  public static final double deployHoldVolts = -0.4;
  public static final double deployPushAlgaeVolts = 0.4;
  public static final double deployStowVolts = -1.5;
  public static final double deployPullBackFF = 1.8; // 1.788545;

  public static final double kAlgaeCaptureKp = 0.1;
  public static final double kAlgaeCaptureKd = 0.0;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployInitalAngle = 45.5; // Degrees
  public static final double algaeStowPosition = 78; // Degress
  public static final double MIN_ANGLE = 45.429;
  public static final double MAX_ANGLE = 135.0;

  public static enum algaeIntakeState {
    OFF,
    PUSH,
    BURST,
    PULL,
    DETECT,
    CAPTURE,
    PULL_ARM,
    STOW,
    CLIMB,
    MANUAL
  }
}
