package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 33;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 50;
  public static final int algaeCaptureCurrentLimit = 50;

  public static final double algaeInSpeed = 0.50;
  public static final double algaeOutSpeed = -1.0;

  public static final double kAlgaeDeployKp = 1.0;
  public static final double kAlgaeDeployKd = 0.04;
  public static final double kAlgaeDeployVel = 100;
  public static final double kAlgaeDeployAcc = 200;
  public static final double kAlgaeDeployAllowableError = .01;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployKf = .924; // Feedforward gain for gravity compensation
  public static final double algaeDeployPosition = 135.0; // Degrees
  public static final double kAlgaeDeployInitalAngle = 59.0; // Degrees
  public static final double algaeStowPosition = 65.0; // Degress
  public static final double MIN_ANGLE = 58.9;
  public static final double MAX_ANGLE = 140.0;
}
