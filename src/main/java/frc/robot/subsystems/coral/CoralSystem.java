package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralSystemPresets.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.LinearFilter;
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
import frc.robot.util.Elastic;
import frc.robot.util.ReefScoringLogger;
import frc.robot.util.RobotStatus;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralSystem extends SubsystemBase {

  public static enum CoralSystemMovementState {
    STABLE,
    SAFE_ARM,
    MOVE_ELEVATOR,
    MOVE_ARM_FINAL,
    MOVE_SIMULTANEOUS,
    ARM_RECOVERY,
    ELEVATOR_RECOVERY
  }

  // Enum representing the states for coral pickup
  private enum CoralPickupState {
    WAITING_FOR_CORAL,
    HAVE_CORAL_SAFE_DISTANCE_FROM_STATION,
    HAVE_CORAL_NEAR_STATION,
  }

  private static final double TIME_OF_FLIGHT_THRESHOLD = .7; // meters
  private double SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE = .7; // meters

  private CANrange canRange = new CANrange(31);

  @Getter private Elevator elevator;
  @Getter public final CoralSystemPresetChooser coralSystemPresetChooser;
  @Getter private Arm arm;
  @Getter private Intake intake;

  @Getter public boolean haveCoral;

  @AutoLogOutput(key = "CoralSystem/climbASAP")
  @Getter
  public boolean climbASAP = false;

  @Getter
  @AutoLogOutput(key = "CoralSystem/justMissed")
  public boolean justMissedCoralScore;

  @Getter
  @AutoLogOutput(key = "CoralSystem/justScored")
  public boolean justScoredCoral;

  @Getter
  @AutoLogOutput(key = "CoralSystem/justPickedUpCoral")
  public boolean justPickedUpCoral;

  @AutoLogOutput(key = "CoralSystem/targetCoralPreset")
  @Getter
  public CoralSystemPresets targetCoralPreset =
      CoralSystemPresets.STARTUP; // Default startup position

  @AutoLogOutput(key = "CoralSystem/currentCoralPreset")
  @Getter
  public CoralSystemPresets currentCoralPreset =
      CoralSystemPresets.STARTUP; // Tracks last reached preset

  // Current state of the pickup state machine
  @AutoLogOutput(key = "CoralSystem/coralPickupState")
  @Getter
  private CoralPickupState coralPickupState = CoralPickupState.WAITING_FOR_CORAL;

  // Field to track the previous coral state
  private boolean previousHaveCoral = false;

  // Should be move are safe or simultanious when responding to button press
  private boolean moveArmSafely = true;

  // For calculating a moving average of the TOF sensor readings
  private final LinearFilter tofFilter = LinearFilter.movingAverage(10);

  @AutoLogOutput(key = "CoralSystem/coralSystemState")
  @Getter
  private CoralSystemMovementState coralSystemState = CoralSystemMovementState.STABLE;

  private double setArmIntialAngleTries = 0;

  public CoralSystem(Elevator elevator, Arm arm, Intake intake) {
    coralSystemPresetChooser = new CoralSystemPresetChooser();
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;

    // Configure the CANrange for basic use
    CANrangeConfiguration configs = new CANrangeConfiguration();

    // Write these configs to the CANrange
    canRange.getConfigurator().apply(configs);

    // Puts the button/command on the dashboard to go to the choosed preset
    SmartDashboard.putData("Set Coral Config", CoralSystemCommands.runPreset(this)); //
  }

  @Override
  public void periodic() {

    // See if we can set the arm inital angle
    if (setArmIntialAngleTries <= 100) {
      setArmIntialAngleTries++;
      this.arm.setInitialAngle(this.intake.get_Arm_TBE_DEG());
    }

    this.elevator.periodic();
    this.arm.periodic();
    this.intake.periodic();

    haveCoral = this.intake.haveCoral();
    ReefScoringLogger.checkAndLogScoringEvent(RobotStatus.getRobotPose(), this);

    Logger.recordOutput("CoralSystem/HaveCoral", haveCoral);
    Logger.recordOutput("CoralSystem/ElevatorAtGoal", elevator.isAtGoal());
    Logger.recordOutput("CoralSystem/ArmAtGoal", arm.isAtGoal());
    Logger.recordOutput("CoralSystem/AtGoal", isAtGoal());

    CoralRPStatusLogger.logCoralStatus(false);

    if (DriverStation.isEnabled()) {
      updateCoralPickupState();
    }

    switch (coralSystemState) {
      case STABLE:
        // Do Nothing
        if (climbASAP) deployClimberTriggered();
        break;
      case SAFE_ARM:
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
        this.elevator.setTargetPreset(targetCoralPreset);
        // Make sure the intake is running as we're going to pickup.
        if (targetCoralPreset == CoralSystemPresets.PICKUP
            || targetCoralPreset == CoralSystemPresets.PICKUPFAR) {
          intake.pullCoral();
        }
        if (arm.isAtGoal() && elevator.isAtGoal()) {
          currentCoralPreset = targetCoralPreset;
          coralSystemState = CoralSystemMovementState.STABLE;
        }
        break;
      case ARM_RECOVERY:
        this.arm.recoverArm(); // go to L1 Angle
        if (arm.isAtGoal()) {
          coralSystemState = CoralSystemMovementState.ELEVATOR_RECOVERY;
        }
        break;
      case ELEVATOR_RECOVERY:
        this.elevator.recoverElevator();
        if (elevator.isAtGoal()) {
          elevator.clearElevatorError();
          arm.clearArmError();
          this.targetCoralPreset = CoralSystemPresets.L1;
          this.currentCoralPreset = CoralSystemPresets.L1;
          coralSystemState = CoralSystemMovementState.STABLE;
          Elastic.selectTab("Teleoperated");
          intake.stopIntake();
        }
        break;
      default:
        // do nothing
        break;
    }

    // check the score timer and stop the intake if its greater than a score time
    // threshold
  }

  public void setTargetPreset(CoralSystemPresets requestedPreset) {
    // If the ARM isn't Stuck then allow new presets
    if (arm.isArmInError() == false) {
      // We are trying to go to a different location AND
      // We ARE NOT Climbing AND
      // We are not currently traveling to a location
      if (requestedPreset != this.currentCoralPreset
          && targetCoralPreset != CoralSystemPresets.CLIMB
          && coralSystemState == CoralSystemMovementState.STABLE) {
        // We are allowed to move
        // Lets just hard code situations where we can just move the
        // elevator and arm at the same time
        moveArmSafely = true;
        this.targetCoralPreset = requestedPreset;
        if (this.targetCoralPreset == CoralSystemPresets.PICKUP
            && DriverStation.isAutonomousEnabled()
            && !haveCoral
            && (currentCoralPreset == CoralSystemPresets.L4)) {
          moveArmSafely = false;
        }

        // changes for autonomous only
        if (currentCoralPreset == CoralSystemPresets.PICKUP
            && this.targetCoralPreset == CoralSystemPresets.PREL4) {
          moveArmSafely = false;
        }

        // changes for autonomous only
        if (currentCoralPreset == CoralSystemPresets.PREL4
            && this.targetCoralPreset == CoralSystemPresets.L4) {
          moveArmSafely = false;
        }

        if (currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2
            && targetCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2) {
          moveArmSafely = false;
        }

        if (currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1
            && targetCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1) {
          moveArmSafely = false;
        }

        if (moveArmSafely) {
          // VERY SPEICAL CODE FOR AUTO INITAL PATH
          // changes for autonomous only
          if (currentCoralPreset == CoralSystemPresets.STARTUP
              && this.targetCoralPreset == CoralSystemPresets.L4) {
            // Start Moving Arm to First Path Arm Angle
            this.arm.setTargetPreset(CoralSystemPresets.AUTO_START_L4);
            this.elevator.setTargetPreset(CoralSystemPresets.AUTO_START_L4);
            coralSystemState = CoralSystemMovementState.MOVE_ELEVATOR;
          } else {
            // Start Moving Arm to Safe
            if (this.targetCoralPreset == CoralSystemPresets.PICKUP) {
              this.arm.setTargetPreset(CoralSystemPresets.PRE_PICKUP);
            } else {
              this.arm.setTargetPreset(CoralSystemPresets.ARMSAFE);
            }
            // Change state
            coralSystemState = CoralSystemMovementState.SAFE_ARM;
          }
        } else {
          coralSystemState = CoralSystemMovementState.MOVE_ARM_FINAL;
        }
      }
    }
  }

  public void autoSetHaveCoral(Boolean coral) {
    this.haveCoral = coral;
    this.intake.autoSetHaveCoral(coral);
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

  public boolean preppedForDislodge() {
    return isAtGoal()
        && (currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_1
            || currentCoralPreset == CoralSystemPresets.PREPARE_DISLODGE_PART2_LEVEL_2);
  }

  @AutoLogOutput(key = "CoralSystem/Rear TOF")
  public double getTimeOfFlightRange() {
    return canRange.getDistance().getValueAsDouble();
  }

  public void scoreCoral() {
    if (targetCoralPreset != CLIMB) {
      intake.pushCoral();
    }
  }

  public void recoverArmAndElevator() {
    if (arm.isArmInError()) coralSystemState = CoralSystemMovementState.ARM_RECOVERY;
  }

  public void updateCoralPickupState() {
    double currentTOFAvg = tofFilter.calculate(getTimeOfFlightRange());
    double distanceFromStation = RobotStatus.getCoralStationSelection().getAcceptedDistance();
    boolean nearStation = (distanceFromStation < SAFE_DISTANCE_FROM_STATION_AFTER_INTAKE);

    switch (coralPickupState) {
      case WAITING_FOR_CORAL:
        if (!previousHaveCoral && haveCoral) {
          justPickedUpCoral = true;
          justMissedCoralScore = false;
          justScoredCoral = false;
          coralPickupState = CoralPickupState.HAVE_CORAL_NEAR_STATION;
        }
        break;

      case HAVE_CORAL_NEAR_STATION:
      case HAVE_CORAL_SAFE_DISTANCE_FROM_STATION:
        if (checkIfSafeDistanceFromCoralStation(currentTOFAvg, nearStation)) {
          coralPickupState = CoralPickupState.HAVE_CORAL_SAFE_DISTANCE_FROM_STATION;
          justPickedUpCoral = false;
        }

        if (justExpelledCoral()) {
          justPickedUpCoral = false;
          tofFilter.reset();
          coralPickupState = CoralPickupState.WAITING_FOR_CORAL;
        }

        // Update the previous coral state for the next cycle.
        previousHaveCoral = haveCoral;
        break;
      default:
        break;
    }
  }

  private boolean justExpelledCoral() {
    if (previousHaveCoral && !haveCoral) {
      if (BranchAlignmentUtils.getCurrentBranchAlignmentStatus() == BranchAlignmentStatus.GREEN) {
        justScoredCoral = true;
        justMissedCoralScore = false;
      } else {
        justMissedCoralScore = true;
        justScoredCoral = false;
      }
      return true;
    } else return false;
  }

  private boolean checkIfSafeDistanceFromCoralStation(double currentTOFAvg, boolean nearStation) {
    if (!nearStation || currentTOFAvg > TIME_OF_FLIGHT_THRESHOLD) {
      return true;
    } else return false;
  }

  public void deployClimberTriggered() {
    climbASAP = true;
    if (targetCoralPreset != CLIMB) {
      setTargetPreset(CLIMB);
    }
  }

  public boolean isArmInError() {
    return arm.isArmInError();
  }
}
