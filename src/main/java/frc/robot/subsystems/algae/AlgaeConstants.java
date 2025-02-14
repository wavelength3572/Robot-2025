package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 22; // TODO: fix me
  public static final int algaeCaptureCanId = 21; // TODO: fix me
  public static final int algaeDeployCurrentLimit = 50;
  public static final int algaeCaptureCurrentLimit = 50;

  public static final double algaeInSpeed = 0.50;
  public static final double algaeOutSpeed = -0.75;

  public static final double kAlgaeDeployKp = .3;
  public static final double kAlgaeDeployKd = 0.00;
  public static final double kAlgaeDeployVel = 1000;
  public static final double kAlgaeDeployAcc = 1500;
  public static final double kAlgaeDeployAllowableError = .05;

  public static final double kAlgaeDeployGearing = 1.0; // Adjust based on your gear ratio
  public static final double kAlgaeDeployKf = 0.1; // Feedforward gain for gravity compensation
  public static final double algaeDeployPosition = 50.0;
  public static final double algaeStowPosition = 0.0;

  public static final double algaeStartAngle = 0.0; // Default start angle of deploy arm
}
