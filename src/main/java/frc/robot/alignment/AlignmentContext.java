package frc.robot.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.AlignmentUtils;
import lombok.Getter;

public class AlignmentContext {
  @Getter private final Pose2d robotPose;
  @Getter private final double elevatorHeightInches;
  @Getter private final boolean haveCoral;
  @Getter private final boolean climberDeployed;
  @Getter private final AlignmentUtils.ReefFaceSelection reefFaceSelection;
  @Getter private final AlignmentUtils.CoralStationSelection coralStationSelection;
  @Getter private final AlignmentUtils.CageSelection cageSelection;

  public AlignmentContext(
      Pose2d robotPose, // generally useful
      boolean haveCoral,
      double elevatorHeightInches,
      boolean climberDeployed, // needed for cage alignment
      AlignmentUtils.ReefFaceSelection reefFaceSelection, // needed for reef alignment
      AlignmentUtils.CoralStationSelection coralStationSelection, // needed for cage alignment
      AlignmentUtils.CageSelection cageSelection // needed for cage alignment
      ) {
    this.robotPose = robotPose;
    this.haveCoral = haveCoral;
    this.elevatorHeightInches = elevatorHeightInches;
    this.climberDeployed = climberDeployed;
    this.reefFaceSelection = reefFaceSelection;
    this.coralStationSelection = coralStationSelection;
    this.cageSelection = cageSelection;
  }
}
