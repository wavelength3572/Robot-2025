package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 33;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 50;
  public static final int algaeCaptureCurrentLimit = 50;

  public static final double algaeInSpeed = 0.50;
  public static final double algaeOutSpeed = -0.5;

  public static final double kAlgaeDeployKp = 0.0;
  public static final double kAlgaeDeployKd = 0.0;
  public static final double kAlgaeDeployVel = 250;
  public static final double kAlgaeDeployAcc = 250;
  public static final double kAlgaeDeployAllowableError = .1;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployKf = .924; // Feedforward gain for gravity compensation
  public static final double algaeDeployPosition = 100.0; // Degrees
  public static final double kAlgaeDeployInitalAngle = 59.0; // Degrees
  public static final double algaeStowPosition = 59.0; // Degress
  public static final double MIN_ANGLE = 58.9;
  public static final double MAX_ANGLE = 140.0;
}
