package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.intake.Intake;
import frc.robot.util.CoralRPStatusLogger;
import frc.robot.util.ReefScoringLogger;
import frc.robot.util.RobotStatus;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  private double SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE = 1.5;
  private double NEAR_REEF_DISTANCE = 1;

  @Getter private Elevator elevator;
  @Getter public final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter public boolean coralInRobot;
  @Getter public boolean justNearReef;
  @Getter public boolean justPickedUp;
  @Getter public boolean justScored;

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

    this.arm.setInitialAngle(this.intake.get_Arm_TBE_DEG());

    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this));
  }

  @Override
  public void periodic() {
    this.elevator.periodic();
    this.arm.periodic();
    this.intake.periodic();

    coralInRobot = this.intake.getCoralInRobot();
    ReefScoringLogger.checkAndLogScoringEvent(RobotStatus.getRobotPose(), this);

    Logger.recordOutput("CoralSystem/CoralInRobot", coralInRobot);
    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    CoralRPStatusLogger.logCoralStatus(false);

    boolean safeDistanceFromStation =
        RobotStatus.getCoralStationSelection().getAcceptedDistance()
            > SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE;

    boolean nearReef =
        RobotStatus.getReefFaceSelection().getAcceptedDistance() < NEAR_REEF_DISTANCE;

    Logger.recordOutput("Automation/justPickedUp", justPickedUp);
    justPickedUp = false;

    Logger.recordOutput("Automation/justNearReef", justNearReef);
    justNearReef = false;
    Logger.recordOutput("Automation/justScored", justScored);
    justScored = false;

    if (DriverStation.isTeleop()
        && safeDistanceFromStation
        && coralInRobot
        && currentCoralPreset == PICKUP
        && systemState == CoralSystemMovementState.STABLE) {
      justPickedUp = true;
    }

    if (DriverStation.isTeleop()
        && nearReef
        && coralInRobot
        && currentCoralPreset == STOW
        && systemState == CoralSystemMovementState.STABLE) {
      justNearReef = true;
      justNearReef = false;
    }

    if (DriverStation.isTeleop()
        && !coralInRobot
        && (currentCoralPreset != CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1
            && currentCoralPreset != CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2)
        && (currentCoralPreset == L1
            || currentCoralPreset == L2
            || currentCoralPreset == L3
            || currentCoralPreset == L4)
        && nearReef
        && systemState == CoralSystemMovementState.STABLE) {
      justScored = true;
      justScored = false;
    }

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

    // check the score timer and stop the intake if its greater than a score time
    // threshold
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

  @AutoLogOutput(key = "CoralSystem/Rear TOF")
  public double getTimeOfFlightRange() {
    return timeOfFlight.getRange();
  }

  public void scoreCoral() {
    intake.pushCoral();
    // set state to running coral
    // start a timer
  }
}
