package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class HybridHeadingAlignCommand extends Command {
    private final Drive drive;
    private final Supplier<Rotation2d> targetHeadingSupplier;
    private final DoubleSupplier xInput, yInput;
    private final ProfiledPIDController thetaController;
    private final double speedScale = 0.3;
  
    public HybridHeadingAlignCommand(
        Drive drive,
        Supplier<Rotation2d> targetHeadingSupplier,
        DoubleSupplier xInput,
        DoubleSupplier yInput
    ) {
      this.drive = drive;
      this.targetHeadingSupplier = targetHeadingSupplier;
      this.xInput = xInput;
      this.yInput = yInput;
      this.thetaController = new ProfiledPIDController(
        1.2, 0, 0,
        new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(360)));
      addRequirements(drive);
    }
  
    @Override
    public void initialize() {
      thetaController.reset(drive.getRotation().getRadians());
      thetaController.setGoal(targetHeadingSupplier.get().getRadians());
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
  
    @Override
    public void execute() {
      double x = xInput.getAsDouble() * speedScale;
      double y = yInput.getAsDouble() * speedScale;
      double thetaVel = thetaController.calculate(drive.getRotation().getRadians());
  
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          x, y, thetaVel, drive.getRotation()));
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(thetaController.getPositionError()) < Math.toRadians(1.0)
          && Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) < 0.25;
    }
  }
  