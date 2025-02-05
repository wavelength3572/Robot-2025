package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSystemPresetChooser {
  private final SendableChooser<CoralSystemPresets> chooser = new SendableChooser<>();

  public CoralSystemPresetChooser() {

    // Add all your enum options
    chooser.addOption("Pickup", CoralSystemPresets.PICKUP);
    chooser.addOption("Score Level 1", CoralSystemPresets.SCORE_LEVEL_1);
    chooser.addOption("Score Level 2", CoralSystemPresets.SCORE_LEVEL_2);
    chooser.addOption("Score Level 3", CoralSystemPresets.SCORE_LEVEL_3);
    chooser.addOption("Score Level 4", CoralSystemPresets.SCORE_LEVEL_4);
    chooser.addOption(
        "Front Algae Dislodge Level 1", CoralSystemPresets.FRONT_ALGAE_DISLODGE_LEVEL_1);
    chooser.addOption(
        "Front Algae Dislodge Level 2", CoralSystemPresets.FRONT_ALGAE_DISLODGE_LEVEL_2);

    // Publish the chooser to the dashboard
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  /** Returns the currently selected CoralPosition. */
  public CoralSystemPresets getSelected() {
    return chooser.getSelected();
  }
}
