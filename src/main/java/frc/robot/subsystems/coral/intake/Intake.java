package frc.robot.subsystems.coral.intake;

import org.littletonrobotics.junction.Logger;

public class Intake {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void pushCoral() {
    io.pushCoral();
  }

  public void pullCoral() {
    io.pullCoral();
  }

  public void forcePullCoral() {
    io.forcePullCoral();
  }

  public void stopIntake() {
    io.stopIntake();
  }

  public boolean haveCoral() {
    return io.haveCoral();
  }

  public void autoSetHaveCoral(boolean coral) {
    io.autoSetHaveCoral(coral);
  }

  public double get_Arm_TBE_DEG() {
    return io.get_Arm_TBE_DEG();
  }
}
