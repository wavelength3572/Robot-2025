package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.CoralSystemPresets.CoralState;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.endeffector.EndEffector;
import org.littletonrobotics.junction.Logger;

public class CoralSubsystem extends SubsystemBase {
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;

  private CoralState selectedScoringLevel = CoralState.PREPARE_L4_SCORE; // Default scoring level
  private boolean haveCoral = false;
  private boolean haveAlgae = false;

  private final CoralSubsystemVisualization2d coralSubsystemVisualization2d;
  private final CoralStateMachine stateMachine;

  public CoralSubsystem(Elevator elevator, Arm arm, EndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
    this.stateMachine = new CoralStateMachine(this);

    SmartDashboard.putString("Selected Scoring Level", selectedScoringLevel.name());

    // Create the visualization if enabled.
    coralSubsystemVisualization2d = new CoralSubsystemVisualization2d(elevator, arm);
  }

  @Override
  public void periodic() {
    // Update subcomponents.
    elevator.update();
    arm.update();
    endEffector.update();

    // Refresh 2D visualization if enabled.
    if (coralSubsystemVisualization2d != null) {
      coralSubsystemVisualization2d.update(elevator, arm);
    }

    // Log sensor readings and statuses.
    Logger.recordOutput("Coral/Elevator Actual Height", elevator.getHeightInMeters());
    Logger.recordOutput("Coral/Arm Actual Angle", arm.getAngleInDegrees());
    Logger.recordOutput("Coral/Has Coral", haveCoral());

    // Update state machine.
    stateMachine.update();

    // update dynamic poses for 3D visualization.
    double elevatorOffset = elevator.getHeightInMeters();
    Pose3d elevatorDynamicPose = new Pose3d(0, -0.0908, elevatorOffset + 0.24, new Rotation3d(0, 0, 0));
    double armAngleRadians = Math.toRadians(arm.getCalibratedAngleDegrees());
    Pose3d armDynamicPose =new Pose3d(0, -0.190, elevatorOffset + 0.262, new Rotation3d(0, armAngleRadians, Units.degreesToRadians(180)));
    Logger.recordOutput("FinalComponentPoses", new Pose3d[] {elevatorDynamicPose, armDynamicPose});
  }

  public boolean haveCoral() {return haveCoral;}
  public void setHaveCoral() {this.haveCoral = true;}  
  public void setDoNotHaveCoral() {this.haveCoral = false;}

  public boolean haveAlgae() {return haveAlgae;}
  public void setHaveAlgae() {this.haveAlgae = true;}
  public void setDoNotHaveAlgae() {this.haveAlgae = false;}

  public void setSelectedScoringLevel(CoralState level) {
    if (level == CoralState.L1_SCORE
        || level == CoralState.L2_SCORE
        || level == CoralState.L3_SCORE
        || level == CoralState.L4_SCORE) {
      selectedScoringLevel = level;
      SmartDashboard.putString("Selected Scoring Level", level.name());
    }
  }

  public CoralState getSelectedScoringLevel() {
    return selectedScoringLevel;
  }

  public void cycleScoringLevel(boolean forward) {
    CoralState[] scoringLevels = {
      CoralState.PREPARE_L1_SCORE,
      CoralState.PREPARE_L2_SCORE,
      CoralState.PREPARE_L3_SCORE,
      CoralState.PREPARE_L4_SCORE
    };

    int currentIndex = 0;
    for (int i = 0; i < scoringLevels.length; i++) {
      if (scoringLevels[i] == selectedScoringLevel) {
        currentIndex = i;
        break;
      }
    }

    if (forward) {
      selectedScoringLevel = scoringLevels[(currentIndex + 1) % scoringLevels.length];
    } else {
      selectedScoringLevel =
          scoringLevels[(currentIndex - 1 + scoringLevels.length) % scoringLevels.length];
    }

    SmartDashboard.putString("Selected Scoring Level", selectedScoringLevel.name());
    System.out.println("🔄 Scoring Level Changed: " + selectedScoringLevel);
  }

  public void cycleScoringLevelForward() {
    cycleScoringLevel(true);
  }

  public void cycleScoringLevelBackward() {
    cycleScoringLevel(false);
  }

  //Getters
  public Elevator getElevator() {return elevator;}
  public Arm getArm() {return arm;}
  public EndEffector getEndEffector() {return endEffector;}
  public CoralStateMachine getStateMachine() {return stateMachine;}
}
