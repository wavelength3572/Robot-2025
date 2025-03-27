package frc.robot.commands.Alignment.TwoStage;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FinalAlign extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final HolonomicDriveController controller;
  private int stableCycles = 0;

  public FinalAlign(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;

    PIDController xPID = new PIDController(3.0, 0.0, 0.2);
    PIDController yPID = new PIDController(3.0, 0.0, 0.2);
    ProfiledPIDController thetaPID =
        new ProfiledPIDController(
            2.0,
            0.0,
            0.1,
            new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(180)));

    xPID.setTolerance(0.01);
    yPID.setTolerance(0.01);
    thetaPID.setTolerance(Math.toRadians(1.5));

    controller = new HolonomicDriveController(xPID, yPID, thetaPID);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    Pose2d current = drive.getPose();

    Trajectory.State dummyState = new Trajectory.State();
    dummyState.poseMeters = targetPose;
    dummyState.velocityMetersPerSecond = 0.0;
    dummyState.accelerationMetersPerSecondSq = 0.0;

    ChassisSpeeds speeds = controller.calculate(current, dummyState, targetPose.getRotation());

    drive.runVelocity(speeds); // or field-relative version if needed

    if (controller.atReference()) {
      stableCycles++;
    } else {
      stableCycles = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return stableCycles > 3;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
