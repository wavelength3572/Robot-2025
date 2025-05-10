package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseJoystickCancelAfterArmSafe extends DriveToPoseJoystickCancel {

  private final CoralSystem coralSystem;

  public DriveToPoseJoystickCancelAfterArmSafe(
      Drive drivetrain,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xJoystickSupplier,
      DoubleSupplier yJoystickSupplier,
      DoubleSupplier rotationJoystickSupplier,
      CoralSystem coralSystem) {
    super(drivetrain, poseSupplier, xJoystickSupplier, yJoystickSupplier, rotationJoystickSupplier);
    this.coralSystem = coralSystem;
  }

  @Override
  public boolean isFinished() {
    if (!coralSystem.armIsPastSafeThresholdForAlgaeDislodge()) {
      Logger.recordOutput("DriveToPoseJoystick/ArmSafe", false);
      return super.atGoal(); // allow normal finish only
    }

    Logger.recordOutput("DriveToPoseJoystick/ArmSafe", true);

    // Allow joystick-based cancel once arm is safe
    if (joystickMoved()) {
      Logger.recordOutput("DriveToPoseJoystick/CanceledByJoystick", true);
      return true;
    }

    return super.isFinished();
  }

  private boolean joystickMoved() {
    return Math.abs(xJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(yJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND
        || Math.abs(rotationJoystickSupplier.getAsDouble()) > JOYSTICK_DEADBAND;
  }
}
