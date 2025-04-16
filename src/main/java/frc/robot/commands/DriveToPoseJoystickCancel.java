package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseJoystickCancel extends DriveToPose {
  public static final double JOYSTICK_DEADBAND = 0.1;

  public final DoubleSupplier xJoystickSupplier;
  public final DoubleSupplier yJoystickSupplier;
  public final DoubleSupplier rotationJoystickSupplier;

  public DriveToPoseJoystickCancel(
      Drive drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier) {
    super(drivetrain, poseSupplier);
    this.xJoystickSupplier = xJoystickSupplier;
    this.yJoystickSupplier = yJoystickSupplier;
    this.rotationJoystickSupplier = rotationJoystickSupplier;
  }

  @Override
  public boolean isFinished() {
    // Cancel if joystick moved beyond deadband
    if (Math.abs(xJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(yJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(rotationJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND) {
      Logger.recordOutput("DriveToPoseJoystick/CanceledByJoystick", true);
      return true;
    }

    // Otherwise use parent logic
    return super.isFinished();
  }
}
