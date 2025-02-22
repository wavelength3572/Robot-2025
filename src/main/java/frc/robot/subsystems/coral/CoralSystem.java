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
import java.util.ArrayDeque;
import java.util.Deque;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  private double SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE = 1.5;
  private double NEAR_REEF_DISTANCE = 1;
  private static final double TIME_OF_FLIGHT_THRESHOLD = 1250; // adjust this constant as needed
  private static final int MOVING_AVG_WINDOW = 10;

  @Getter private Elevator elevator;
  @Getter public final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter public boolean coralInRobot;
  @Getter public boolean justNearReef;
  @Getter public boolean justScored;

  @Getter
  @AutoLogOutput(key = "CoralStation/safeToMoveArm")
  public boolean safeToMoveArmAfterPickupFromStation;

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

  // Enum representing the states for coral pickup
  private enum CoralPickupState {
    WAITING_FOR_CORAL,
    CHECK_IF_NEAR_CORAL_STATION,
    RECEIVED_CORAL_NEAR_CORAL_STATION,
    HAVE_CORAL_WAITING
  }

  // Current state of the pickup state machine
  private CoralPickupState pickupState = CoralPickupState.WAITING_FOR_CORAL;

  // Field to track the previous coral state
  private boolean previousCoralInRobot = false;

  // For calculating a moving average of the TOF sensor readings
  private final Deque<Double> tofReadings = new ArrayDeque<>();

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

    if (DriverStation.isEnabled()) {
      automationTriggerChecks();
    }

    checkAndStowOnCoralPickup();

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

  private void automationTriggerChecks() {

    boolean nearReef =
        RobotStatus.getReefFaceSelection().getAcceptedDistance() < NEAR_REEF_DISTANCE;

    Logger.recordOutput("Automation/justNearReef", justNearReef);
    justNearReef = false;
    Logger.recordOutput("Automation/justScored", justScored);
    justScored = false;

    if (DriverStation.isTeleop()
        && nearReef
        && coralInRobot
        && currentCoralPreset == STOW
        && systemState == CoralSystemMovementState.STABLE) {
      justNearReef = true;
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
    }
  }

  public void setTargetPreset(CoralSystemPresets preset) {
    if (preset != this.currentCoralPreset && targetCoralPreset != CLIMB) {
      this.targetCoralPreset = preset;
      // Start Moving Arm to Safe
      this.arm.setTargetPreset(CoralSystemPresets.STOW);
      // Change state
      systemState = CoralSystemMovementState.SAFE_ARM;
    }
  }

  public void setAlgaeDislodgePreset(CoralSystemPresets preset) {
    if (preset != this.currentCoralPreset && targetCoralPreset != CLIMB) {
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
        "CoralSystem/CoralPickupState",
        "State: " + pickupState + " | coralInRobot: " + coralInRobot);

    // Always update the TOF moving average on every check
    addTOFReading(getTimeOfFlightRange());
    double currentTOFAvg = getTOFMovingAverage();
    double distanceFromStation = RobotStatus.getCoralStationSelection().getAcceptedDistance();
    boolean nearStation = (distanceFromStation < SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE);

    switch (pickupState) {
      case WAITING_FOR_CORAL:
        // Look for a transition from not having coral to having coral.
        if (!previousCoralInRobot && coralInRobot) {
          Logger.recordOutput(
              "CoralPickupState",
              "Coral just received. Transitioning to CHECK_IF_NEAR_CORAL_STATION.");
          pickupState = CoralPickupState.CHECK_IF_NEAR_CORAL_STATION;
        }
        break;

      case CHECK_IF_NEAR_CORAL_STATION:
        // Check if the robot was near the coral station when the coral was picked up.
        Logger.recordOutput(
            "CoralPickupState",
            "Distance from station: " + distanceFromStation + " | nearStation: " + nearStation);

        if (nearStation && currentCoralPreset == PICKUP) {
          Logger.recordOutput(
              "CoralPickupState", "Transitioning to RECEIVED_CORAL_NEAR_CORAL_STATION.");
          pickupState = CoralPickupState.RECEIVED_CORAL_NEAR_CORAL_STATION;
        } else {
          Logger.recordOutput(
              "CoralPickupState", "Not near coral station. Transitioning to HAVE_CORAL_WAITING.");
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
                "CoralPickupState",
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
        if (!coralInRobot) {
          safeToMoveArmAfterPickupFromStation = false;
          Logger.recordOutput(
              "CoralPickupState", "Coral lost. Transitioning back to WAITING_FOR_CORAL.");
          tofReadings.clear(); // Clear readings for the next cycle
          pickupState = CoralPickupState.WAITING_FOR_CORAL;
        }
        break;

      default:
        Logger.recordOutput("CoralPickupState", "Unknown state encountered.");
        break;
    }

    // Update the previous coral state for the next cycle.
    previousCoralInRobot = coralInRobot;
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
  @AutoLogOutput(key = "CoralStation/TOFAverage")
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
