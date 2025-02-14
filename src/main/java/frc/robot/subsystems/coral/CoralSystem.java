package frc.robot.subsystems.coral;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter public final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter public boolean coralInRobot;

  private TimeOfFlight timeOfFlight = new TimeOfFlight(31); // Back of Robot on Elevator

  @AutoLogOutput(key = "CoralSystem/targetCoralPreset")
  @Getter
  public CoralSystemPresets targetCoralPreset =
      CoralSystemPresets.STARTUP; // Default startup position

  @AutoLogOutput(key = "CoralSystem/currentCoralPreset")
  @Getter
  public CoralSystemPresets currentCoralPreset =
      CoralSystemPresets.STARTUP; // Tracks last reached preset

  private enum CoralSystemMovementState {
    STABLE,
    SAFE_ARM,
    MOVE_ELEVATOR,
    MOVE_ARM_FINAL,
    MOVE_SIMULTANEOUS
  }

  private CoralSystemMovementState systemState = CoralSystemMovementState.STABLE;

  public CoralSystem(Elevator elevator, Arm arm, Intake intake) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    timeOfFlight.setRangingMode(RangingMode.Short, 20);

    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this));
  }

  @Override
  public void periodic() {
    this.elevator.periodic();
    this.arm.periodic();
    this.intake.periodic();

    coralInRobot = this.intake.getCoralInRobot();

    Logger.recordOutput("CoralSystem/CoralInRobot", coralInRobot);
    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    switch (systemState) {
      case STABLE:
        // Do Nothing
        break;
      case SAFE_ARM:
        // Move Arm to Safe
        this.arm.setTargetPreset(CoralSystemPresets.STOW);
        if (arm.isAtGoal()) {
          systemState = CoralSystemMovementState.MOVE_ELEVATOR;
          // Start moving elevator
          this.elevator.setTargetPreset(targetCoralPreset);
        }
        break;
      case MOVE_ELEVATOR:
        this.elevator.setTargetPreset(targetCoralPreset);
        if (elevator.isAtGoal()) {
          systemState = CoralSystemMovementState.MOVE_ARM_FINAL;
          // Start Moving arm
          this.arm.setTargetPreset(targetCoralPreset);
        }
        break;
      case MOVE_ARM_FINAL:
        this.arm.setTargetPreset(targetCoralPreset);
        if (arm.isAtGoal()) {
          currentCoralPreset = targetCoralPreset;
          systemState = CoralSystemMovementState.STABLE;
        }
        break;

      case MOVE_SIMULTANEOUS:
        this.arm.setTargetPreset(targetCoralPreset);
        this.elevator.setTargetPreset(targetCoralPreset);
        if (arm.isAtGoal() && elevator.isAtGoal()) {
          currentCoralPreset = targetCoralPreset;
          systemState = CoralSystemMovementState.STABLE;
        }
        break;

      default:
        // do nothing
        break;
    }


    //check the score timer and stop the intake if its greater than a score time threshold
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    if (preset != this.currentCoralPreset) {
      this.targetCoralPreset = preset;
      // Start Moving Arm to Safe
      this.arm.setTargetPreset(CoralSystemPresets.STOW);
      // Change state
      systemState = CoralSystemMovementState.SAFE_ARM;
    }
  }

  public void setAlgaeDislodgePreset(CoralSystemPresets preset) {
    if (preset != this.currentCoralPreset) {
      this.targetCoralPreset = preset;
      systemState = CoralSystemMovementState.MOVE_SIMULTANEOUS;
    }
  }

  public void setCoralInRobot(Boolean coralInRobot) {
    this.coralInRobot = coralInRobot;
    this.intake.setCoralInRobot(coralInRobot);
  }

  public boolean isAtGoal() {
    return systemState == CoralSystemMovementState.STABLE
        && currentCoralPreset == targetCoralPreset;
  }

  @AutoLogOutput(key = "TOF")
  public double getTimeOfFlightRange() {
    return timeOfFlight.getRange();
  }

  public void scoreCoral()
  {
    intake.pushCoral();
    //set state to running coral
    //start a timer

  }

}
