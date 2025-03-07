package frc.robot.subsystems.coral.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.RobotStatus;
import java.util.Map;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

public class IntakeIOVirtualSim implements IntakeIO {

  private Double requestedSpeed = 0.0;
  private boolean haveCoral = false; // Start without a coral
  private static final double MIN_FEED_TIME = 0.50; // Min delay before coral enters
  private static final double MAX_FEED_TIME = 1.0; // Max delay before coral enters
  private static final double EJECTION_TIME =
      0; // Fixed push-out time - I believe this can be 0 because with real robot we count coral as
  // gone as soon as we start pushing
  private static final double FEEDER_THRESHOLD =
      .8; // Robot must be within 0.5 meters to receive coral

  private enum IntakeState {
    OFF,
    PUSH,
    PULL
  }

  private IntakeState currentIntakeState = IntakeState.OFF;
  private final Random random = new Random();
  private final Timer pullTimer = new Timer(); // Timer for coral pulling delay
  private final Timer pushTimer = new Timer(); // Timer for coral ejection
  private double randomPullDelay = 0; // Stores random delay time for pulling coral
  private boolean pullTimerStarted = false; // Flag to track if timer started

  public IntakeIOVirtualSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Simulate pushing - ejecting the coral after a fixed time
    if (currentIntakeState == IntakeState.PUSH) {
      if (pushTimer.hasElapsed(EJECTION_TIME)) {
        haveCoral = false; // Coral fully exits after the ejection time
        currentIntakeState = IntakeState.OFF; // Stop pushing
      }
    }

    // Simulate pulling - checking if we're near the feeder station and waiting for the delay
    if (currentIntakeState == IntakeState.PULL) {
      if (isNearCorrectFeeder() && RobotStatus.getCurrentPreset() == CoralSystemPresets.PICKUP) {
        if (!pullTimerStarted) {
          // Start a random delay before we "receive" the coral
          randomPullDelay = MIN_FEED_TIME + (random.nextDouble() * (MAX_FEED_TIME - MIN_FEED_TIME));
          Logger.recordOutput("AutoTiming/FakeCoralIntakeDelay", randomPullDelay);
          pullTimer.reset();
          pullTimer.start();
          pullTimerStarted = true;
        }

        // After delay, simulate limit switch detecting coral
        if (pullTimer.hasElapsed(randomPullDelay)) {
          haveCoral = true; // Coral enters the robot after delay
          currentIntakeState = IntakeState.OFF; // Stop pulling
          pullTimerStarted = false; // Reset timer flag
        }
      } else {
        // If we are not near the feeder station, reset the timer
        pullTimer.reset();
        pullTimerStarted = false;
      }
    }

    inputs.requestedSpeed = this.requestedSpeed;
    inputs.haveCoral = haveCoral;
  }

  @Override
  public void pullCoral() {
    currentIntakeState = IntakeState.PULL;
    setSpeed(0.5); // Allow the intake to turn on immediately
  }

  @Override
  public void pushCoral() {
    currentIntakeState = IntakeState.PUSH;
    pushTimer.reset();
    pushTimer.start();
    setSpeed(-0.75);
  }

  @Override
  public void stopIntake() {
    currentIntakeState = IntakeState.OFF;
    setSpeed(0.0);
  }

  @Override
  public boolean haveCoral() {
    return haveCoral;
  }

  @Override
  public void setSpeed(double speed) {
    this.requestedSpeed = speed;
  }

  @Override
  public void autoSetHaveCoral(boolean haveCoral) {
    this.haveCoral = haveCoral;
  }

  private boolean isNearCorrectFeeder() {
    Pose2d robotPose = RobotStatus.getRobotPose();
    Translation2d robotTranslation = robotPose.getTranslation();

    // Determine the correct set of feeder tags based on alliance color
    Map<Integer, Translation2d> correctFeederTags =
        (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            ? FieldConstants.RED_CORALSTATION_APRIL_TAGS
            : FieldConstants.BLUE_CORALSTATION_APRIL_TAGS;

    // Check if robot is near any correct feeder station tag
    for (Translation2d feederPosition : correctFeederTags.values()) {
      double distanceToFeeder = robotTranslation.getDistance(feederPosition);
      if (distanceToFeeder < FEEDER_THRESHOLD) {
        return true;
      }
    }

    return false;
  }
}
