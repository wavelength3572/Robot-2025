package frc.robot.subsystems.coral.arm;

public class ArmIOSpark implements Arm.ArmIO {
  @Override
  public void updateInputs() {
    // update sensor readings, etc.
  }

  @Override
  public double getAngle() {
    // return the current angle
    return 0.0;
  }

  @Override
  public void setAngle(double angle) {
    // command the motor to move toward the specified angle
  }
}
