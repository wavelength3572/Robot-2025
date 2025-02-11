package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSystemPresetChooser {
  private SendableChooser<CoralSystemPresets> chooser = new SendableChooser<>();

  public CoralSystemPresetChooser() {
    setupChooser();
  }

  private void setupChooser() {
    chooser = new SendableChooser<>(); // Clear previous options
    chooser.setDefaultOption("Stow", CoralSystemPresets.STOW);
    chooser.addOption("Pickup", CoralSystemPresets.PICKUP);
    chooser.addOption("L1", CoralSystemPresets.SCORE_LEVEL_1);
    chooser.addOption("L2", CoralSystemPresets.SCORE_LEVEL_2);
    chooser.addOption("L3", CoralSystemPresets.SCORE_LEVEL_3);
    chooser.addOption("L4", CoralSystemPresets.SCORE_LEVEL_4);
    chooser.addOption("Dislodge L1", CoralSystemPresets.DISLODGE_LEVEL_1);
    chooser.addOption("Dislodge L2", CoralSystemPresets.DISLODGE_LEVEL_2);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  public CoralSystemPresets getSelected() {
    return chooser.getSelected();
  }
}
