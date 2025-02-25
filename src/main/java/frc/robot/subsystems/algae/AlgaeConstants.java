package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 33;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 30;
  public static final int algaeCaptureCurrentLimit = 30;

  public static final double algaeInSpeed = 0.50;
  public static final double algaeOutSpeed = -1.0;

  public static final double kAlgaeDeployKp = 0.0;
  public static final double kAlgaeDeployKd = 0.00;
  public static final double kAlgaeDeployVel = 100;
  public static final double kAlgaeDeployAcc = 200;
  public static final double kAlgaeDeployAllowableError = .01;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployKf = .924; // Feedforward gain for gravity compensation
  public static final double algaeDeployPosition = 135.0; // Degrees
  public static final double kAlgaeDeployInitalAngle = 45.5; // Degrees
  public static final double algaeStowPosition = 78; // Degress
  public static final double MIN_ANGLE = 45.429;
  public static final double MAX_ANGLE = 135.0;
}
