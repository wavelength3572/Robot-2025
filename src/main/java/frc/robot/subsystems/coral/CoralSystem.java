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
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.BranchAlignmentUtils.BranchAlignmentStatus;
import frc.robot.util.CoralRPStatusLogger;
import frc.robot.util.ReefScoringLogger;
import frc.robot.util.RobotStatus;
import java.util.ArrayDeque;
import java.util.Deque;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  public static enum CoralSystemMovementState {
    STABLE,
    SAFE_ARM,
    MOVE_ELEVATOR,
    MOVE_ARM_FINAL,
    MOVE_SIMULTANEOUS
  }

  // Enum representing the states for coral pickup
  private enum CoralPickupState {
    WAITING_FOR_CORAL,
    CHECK_IF_NEAR_CORAL_STATION,
    RECEIVED_CORAL_NEAR_CORAL_STATION,
    HAVE_CORAL_WAITING
  }

  private TimeOfFlight timeOfFlight = new TimeOfFlight(31); // Back of Robot on Elevator
  private static final double TIME_OF_FLIGHT_THRESHOLD = 1250; // adjust this constant as needed

  private double SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE = 1.5;
  private static final int MOVING_AVG_WINDOW = 10;

  @Getter private Elevator elevator;
  @Getter public final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter public boolean haveCoral;

  @Getter
  @AutoLogOutput(key = "CoralSystem/justMissed")
  public boolean justMissedCoralScoral;

  @Getter
  @AutoLogOutput(key = "CoralSystem/justScored")
  public boolean justScoredCoral;

  @Getter
  @AutoLogOutput(key = "CoralSystem/safeToMoveArmAfterPickupFromStation")
  public boolean safeToMoveArmAfterPickupFromStation;

  @AutoLogOutput(key = "CoralSystem/targetCoralPreset")
  @Getter
  public CoralSystemPresets targetCoralPreset =
      CoralSystemPresets.STARTUP; // Default startup position

  @AutoLogOutput(key = "CoralSystem/currentCoralPreset")
  @Getter
  public CoralSystemPresets currentCoralPreset =
      CoralSystemPresets.STARTUP; // Tracks last reached preset

  // Current state of the pickup state machine
  private CoralPickupState pickupState = CoralPickupState.WAITING_FOR_CORAL;

  // Field to track the previous coral state
  private boolean previousHaveCoral = false;

  // For calculating a moving average of the TOF sensor readings
  private final Deque<Double> tofReadings = new ArrayDeque<>();

  @AutoLogOutput(key = "CoralSystem/coralSystemState")
  @Getter
  private CoralSystemMovementState coralSystemState = CoralSystemMovementState.STABLE;

  public CoralSystem(Elevator elevator, Arm arm, Intake intake) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    timeOfFlight.setRangingMode(RangingMode.Short, 20);

    // try {
    // TimeUnit.SECONDS.sleep(1);
    // } catch (InterruptedException e) {
    // Thread.currentThread().interrupt();
    // }

    this.arm.setInitialAngle(this.intake.get_Arm_TBE_DEG());

    // Puts the button/command on the dashboard to go to the choosed preset
    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this)); //
  }

  @Override
  public void periodic() {

    this.elevator.periodic();
    this.arm.periodic();
    this.intake.periodic();

    haveCoral = this.intake.haveCoral();
    ReefScoringLogger.checkAndLogScoringEvent(RobotStatus.getRobotPose(), this);

    Logger.recordOutput("CoralSystem/HaveCoral", haveCoral);
    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    CoralRPStatusLogger.logCoralStatus(false);

    if (DriverStation.isEnabled()) {
      checkAndStowOnCoralPickup();
    }

    switch (coralSystemState) {
      case STABLE:
        // Do Nothing
        break;
      case SAFE_ARM:
        // Move Arm to Safe
        this.arm.setTargetPreset(CoralSystemPresets.ARMSAFE);
        if (arm.getCurrentAngleDEG()
            >= CoralSystemPresets.ARMSAFE.getArmAngle() - 1.0) { // Put in a 1 degree fudge
          // factor
          coralSystemState = CoralSystemMovementState.MOVE_ELEVATOR;
          // Start moving elevator
          this.elevator.setTargetPreset(targetCoralPreset);
        }
        break;
      case MOVE_ELEVATOR:
        this.elevator.setTargetPreset(targetCoralPreset);
        if (elevator.isAtGoal()) {
          coralSystemState = CoralSystemMovementState.MOVE_ARM_FINAL;
          // Start Moving arm
          this.arm.setTargetPreset(targetCoralPreset);
        }
        break;
      case MOVE_ARM_FINAL:
        this.arm.setTargetPreset(targetCoralPreset);
        if (arm.isAtGoal()) {
          currentCoralPreset = targetCoralPreset;
          coralSystemState = CoralSystemMovementState.STABLE;
        }
        break;

      case MOVE_SIMULTANEOUS:
        this.arm.setTargetPreset(targetCoralPreset);
        this.elevator.setTargetPreset(targetCoralPreset);
        if (arm.isAtGoal() && elevator.isAtGoal()) {
          currentCoralPreset = targetCoralPreset;
          coralSystemState = CoralSystemMovementState.STABLE;
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
    // We are trying to go to a different location AND
    // We ARE NOT Climbing AND
    // We are not currently traveling to a location
    if (preset != this.currentCoralPreset
        && targetCoralPreset != CLIMB
        && coralSystemState == CoralSystemMovementState.STABLE) {
      // We are allowed to move
      this.targetCoralPreset = preset;
      // Start Moving Arm to Safe
      this.arm.setTargetPreset(CoralSystemPresets.STOW);
      // Change state
      coralSystemState = CoralSystemMovementState.SAFE_ARM;
    }
  }

  public void setSimultaneousTargetPreset(CoralSystemPresets preset) {
    // We are trying to go to a different location AND
    // We ARE NOT Climbing AND
    // We are not currently traveling to a location
    if (preset != this.currentCoralPreset
        && targetCoralPreset != CLIMB
        && coralSystemState == CoralSystemMovementState.STABLE) {
      this.targetCoralPreset = preset;
      coralSystemState = CoralSystemMovementState.MOVE_SIMULTANEOUS;
    }
  }

  public void autoSetHaveCoral(Boolean haveCoral) {
    this.haveCoral = haveCoral;
    this.intake.autoSetHaveCoral(haveCoral);
  }

  public boolean isAtGoal() {
    boolean atTargetState =
        coralSystemState == CoralSystemMovementState.STABLE
            && currentCoralPreset == targetCoralPreset;

    boolean preppedForDislodge =
        ((currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
                || currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2)
            && coralSystemState == CoralSystemMovementState.STABLE);

    return atTargetState || preppedForDislodge;
  }

  @AutoLogOutput(key = "CoralSystem/Rear TOF")
  public double getTimeOfFlightRange() {
    return timeOfFlight.getRange();
  }

  public void scoreCoral() {
    if (targetCoralPreset != CLIMB) {
      intake.pushCoral();
      // set state to running coral
      // start a timer
    }
  }

  /**
   * Checks if the robot just picked up a coral while being near the coral station and if the
   * time-of-flight sensor reading (moving average of last 10 values) exceeds a threshold. If so,
   * set the target preset to STOW.
   */
  public void checkAndStowOnCoralPickup() {
    // Log the current state and coral status at the start of the check
    Logger.recordOutput(
        "CoralSystem/CoralPickupStateStatus",
        "State: " + pickupState + " | HaveCoral: " + haveCoral);

    // Always update the TOF moving average on every check
    addTOFReading(getTimeOfFlightRange());
    double currentTOFAvg = getTOFMovingAverage();
    double distanceFromStation = RobotStatus.getCoralStationSelection().getAcceptedDistance();
    boolean nearStation = (distanceFromStation < SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE);

    switch (pickupState) {
      case WAITING_FOR_CORAL:
        // Look for a transition from not having coral to having coral.
        if (!previousHaveCoral && haveCoral) {
          justMissedCoralScoral = false;
          justScoredCoral = false;
          Logger.recordOutput(
              "CoralSystem/CoralPickupState",
              "Coral just received. Transitioning to CHECK_IF_NEAR_CORAL_STATION.");
          pickupState = CoralPickupState.CHECK_IF_NEAR_CORAL_STATION;
        }
        break;

      case CHECK_IF_NEAR_CORAL_STATION:
        // Check if the robot was near the coral station when the coral was picked up.
        Logger.recordOutput(
            "CoralSystem/CoralPickupState",
            "Distance from station: " + distanceFromStation + " | nearStation: " + nearStation);

        if (nearStation && currentCoralPreset == PICKUP) {
          Logger.recordOutput(
              "CoralSystem/CoralPickupState",
              "Transitioning to RECEIVED_CORAL_NEAR_CORAL_STATION.");
          pickupState = CoralPickupState.RECEIVED_CORAL_NEAR_CORAL_STATION;
        } else {
          Logger.recordOutput(
              "CoralSystem/CoralPickupState",
              "Not near coral station. Transitioning to HAVE_CORAL_WAITING.");
          pickupState = CoralPickupState.HAVE_CORAL_WAITING;
        }
        break;

      case RECEIVED_CORAL_NEAR_CORAL_STATION:
        if (currentCoralPreset == PICKUP) {
          // Here, we continuously update the moving average (already updated at the top).
          // Check if the moving average exceeds the threshold.
          if (currentTOFAvg > TIME_OF_FLIGHT_THRESHOLD
              && !nearStation) { // might want to only use TOF if the data is reliable.
            Logger.recordOutput(
                "CoralSystem/CoralPickupState",
                "TOF average ("
                    + currentTOFAvg
                    + ") above threshold ("
                    + TIME_OF_FLIGHT_THRESHOLD
                    + "). Setting target preset to STOW.");
            // setTargetPreset(CoralSystemPresets.STOW);
            safeToMoveArmAfterPickupFromStation = true;
            pickupState = CoralPickupState.HAVE_CORAL_WAITING;
          }
        } else {
          pickupState = CoralPickupState.HAVE_CORAL_WAITING;
        }
        break;

      case HAVE_CORAL_WAITING:
        // Wait until the robot no longer has a coral.
        if (!haveCoral) {

          if (BranchAlignmentUtils.getCurrentBranchAlignmentStatus()
              == BranchAlignmentStatus.GREEN) {
            justScoredCoral = true;
          } else justMissedCoralScoral = true;

          safeToMoveArmAfterPickupFromStation = false;
          Logger.recordOutput(
              "CoralSystem/CoralPickupState",
              "Coral Expelled. Transitioning back to WAITING_FOR_CORAL.");
          tofReadings.clear(); // Clear readings for the next cycle
          pickupState = CoralPickupState.WAITING_FOR_CORAL;
        }
        break;

      default:
        Logger.recordOutput("CoralSystem/CoralPickupState", "Unknown state encountered.");
        break;
    }

    // Update the previous coral state for the next cycle.
    previousHaveCoral = haveCoral;
  }

  // Helper method to add a TOF reading and keep a window of the last
  // MOVING_AVG_WINDOW values
  private void addTOFReading(double reading) {
    if (reading != 0) {
      tofReadings.addLast(reading);
      if (tofReadings.size() > MOVING_AVG_WINDOW) {
        tofReadings.removeFirst();
      }
    }
  }

  // Helper method to compute the current moving average of the TOF sensor values
  @AutoLogOutput(key = "CoralSystem/TOFAverage")
  private double getTOFMovingAverage() {
    double sum = 0.0;
    for (Double val : tofReadings) {
      sum += val;
    }
    return tofReadings.isEmpty() ? 0.0 : sum / tofReadings.size();
  }

  public void deployClimberTriggered() {
    if (targetCoralPreset != CLIMB) {
      setTargetPreset(CLIMB);
    }
  }
}
