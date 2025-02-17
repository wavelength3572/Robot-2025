package frc.robot.subsystems.algae;

public class AlgaeConstants {
  public static final int algaeDeployCanId = 31;
  public static final int algaeCaptureCanId = 30;
  public static final int algaeDeployCurrentLimit = 20;
  public static final int algaeCaptureCurrentLimit = 20;

  public static final double algaeInSpeed = 0.50;
  public static final double algaeOutSpeed = -0.5;

  public static final double kAlgaeDeployKp = 0.0;
  public static final double kAlgaeDeployKd = 0.0;
  public static final double kAlgaeDeployVel = 250;
  public static final double kAlgaeDeployAcc = 500;
  public static final double kAlgaeDeployAllowableError = .1;

  public static final double kAlgaeDeployGearing = 7.5;
  public static final double kAlgaeDeployKf = 0.0; // Feedforward gain for gravity compensation
  public static final double algaeDeployPosition = 1.2;
  public static final double algaeStowPosition = 0.0;
  public static final double MIN_ROTATIONS = 0;
  public static final double MAX_ROTATIONS = 1.5;
}
