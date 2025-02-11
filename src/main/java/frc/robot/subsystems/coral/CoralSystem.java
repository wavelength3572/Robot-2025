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
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter private final CoralSystemPresetChooser coralSystemPresetChooser;

  @Getter public boolean coralInRobot;

  private TimeOfFlight timeOfFlight = new TimeOfFlight(31); // Back of Robot on Elevator

  @AutoLogOutput(key = "CoralSystem/targetCoralPreset")
  @Getter
  private CoralSystemPresets targetCoralPreset =
      CoralSystemPresets.STARTUP; // Default startup position

  @AutoLogOutput(key = "CoralSystem/currentCoralPreset")
  @Getter
  private CoralSystemPresets currentCoralPreset =
      CoralSystemPresets.STARTUP; // Tracks last reached preset

  private enum CoralSystemMovementState {
    MOVE_ARM_TO_SAFETY,
    MOVE_ELEVATOR,
    MOVE_ARM,
    WAIT_FOR_NEW_PRESET
  }

  private CoralSystemMovementState systemState = CoralSystemMovementState.WAIT_FOR_NEW_PRESET;

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

    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    switch (systemState) {
      case MOVE_ARM_TO_SAFETY:
        // Move Arm to Safe
        this.arm.setTargetPreset(CoralSystemPresets.STOW);
        if (arm.isAtGoal()) {
          systemState = CoralSystemMovementState.MOVE_ELEVATOR;
        }
        break;
      case MOVE_ELEVATOR:
        this.elevator.setTargetPreset(targetCoralPreset);
        if (elevator.isAtGoal()) {
          systemState = CoralSystemMovementState.MOVE_ARM;
        }
        break;
      case MOVE_ARM:
        this.arm.setTargetPreset(targetCoralPreset);
        if (arm.isAtGoal()) {
          systemState = CoralSystemMovementState.WAIT_FOR_NEW_PRESET;
          currentCoralPreset = targetCoralPreset;
        }
        break;
      case WAIT_FOR_NEW_PRESET:
      default:
        // do nothing
        break;
    }
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    this.targetCoralPreset = preset;
    systemState = CoralSystemMovementState.MOVE_ARM_TO_SAFETY;
  }

  public void setCoralInRobot(Boolean coralInRobot) {
    this.coralInRobot = coralInRobot;
    this.intake.setCoralInRobot(coralInRobot);
  }

  public boolean isAtGoal() {
    return elevator.isAtGoal() && arm.isAtGoal();
  }

  @AutoLogOutput(key = "TOF")
  public double getTimeOfFlightRange() {
    return timeOfFlight.getRange();
  }
}
