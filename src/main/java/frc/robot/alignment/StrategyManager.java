package frc.robot.alignment;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.concurrent.atomic.AtomicBoolean;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class StrategyManager implements AlignmentStrategy {
  private static final double THRESHOLD_DISTANCE_TO_REEF = 2.0;
  private static final double THRESHOLD_DISTANCE_TO_CORAL_STATION = 2.0;
  private static final double THRESHOLD_DISTANCE_TO_CAGE = 1.0;

  // Shared PID controller for all rotational alignment strategies
  private final ProfiledPIDController sharedAngleController;
  // Shared manual override flag
  private final AtomicBoolean isManualOverride = new AtomicBoolean(false);

  private final AlignmentStrategy reefStrategy;
  private final AlignmentStrategy coralStationStrategy;
  private final AlignmentStrategy cageStrategy;
  private final AlignmentStrategy cageFullAlignmentStrategy;
  private final NoOpAlignmentStrategy noOpStrategy = new NoOpAlignmentStrategy();

  // Store the currently active strategy for consistency across a loop cycle
  private AlignmentStrategy currentActiveStrategy = noOpStrategy;

  @AutoLogOutput(key = "Alignment/useFullCageAlignment")
  @Getter
  private boolean useFullCageAlignment = false; // 🔥 Toggle flag

  public StrategyManager() {
    // Initialize shared PID controller
    this.sharedAngleController =
        new ProfiledPIDController(
            5.0, // ANGLE_KP
            0.0,
            0.4, // ANGLE_KD
            new TrapezoidProfile.Constraints(8.0, 20.0));
    this.sharedAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Instantiate alignment strategies with shared controller and override flag
    reefStrategy = new ReefAlignmentStrategy(sharedAngleController);
    coralStationStrategy = new CoralStationAlignmentStrategy(sharedAngleController);
    cageStrategy = new CageAlignmentStrategy(sharedAngleController);
    cageFullAlignmentStrategy = new CageFullAlignmentStrategy(sharedAngleController);
  }

  public void toggleAutoCageAlignmentMode() {
    useFullCageAlignment = !useFullCageAlignment;
  }

  /**
   * Updates the active alignment strategy based on the robot’s current context. This should be
   * called **once per loop cycle**.
   */
  public void updateStrategyForCycle(AlignmentContext context) {
    boolean haveCoral = context.isCoralInRobot();
    boolean climberDeployed = context.isClimberDeployed();
    boolean nearReef =
        context.getReefFaceSelection().getAcceptedDistance() <= THRESHOLD_DISTANCE_TO_REEF;
    boolean nearCoralStation =
        context.getCoralStationSelection().getAcceptedDistance()
            <= THRESHOLD_DISTANCE_TO_CORAL_STATION;
    boolean nearCage = context.getCageSelection().getDistanceToCage() <= THRESHOLD_DISTANCE_TO_CAGE;

    if (nearReef && haveCoral && !climberDeployed) {
      currentActiveStrategy = reefStrategy;
    } else if (nearCoralStation && !haveCoral && !climberDeployed) {
      currentActiveStrategy = coralStationStrategy;
    } else if (nearCage && climberDeployed) {
      currentActiveStrategy = useFullCageAlignment ? cageFullAlignmentStrategy : cageStrategy;
    } else {
      currentActiveStrategy = noOpStrategy;
    }
  }

  @Override
  public double getRotationalCorrection(AlignmentContext context, double manualRotationInput) {
    // If driver is actively rotating, set override and return manual input.
    if (Math.abs(manualRotationInput) > 0.0) {
      isManualOverride.set(true);
      return manualRotationInput;
    }

    // If driver stopped rotating and manual override is ON, reset PID and turn off
    // the override.
    if (isManualOverride.get()) {
      resetAngleController(context.getRobotPose(), getGoalRotation(context));
      isManualOverride.set(false);
    }
    return currentActiveStrategy.getRotationalCorrection(context, manualRotationInput);
  }

  @Override
  public Translation2d getTranslationalCorrection(
      AlignmentContext context, Translation2d manualTranslation) {
    return currentActiveStrategy.getTranslationalCorrection(context, manualTranslation);
  }

  private void resetAngleController(Pose2d robotPose, double goalRotation) {
    sharedAngleController.reset(robotPose.getRotation().getRadians());
    sharedAngleController.setGoal(goalRotation);
  }

  @Override
  public double getGoalRotation(AlignmentContext context) {
    return currentActiveStrategy.getGoalRotation(context);
  }
}
