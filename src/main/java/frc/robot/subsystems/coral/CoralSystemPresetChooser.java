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
    chooser.addOption("Startup", CoralSystemPresets.STARTUP);
    chooser.addOption("Pickup", CoralSystemPresets.PICKUP);
    chooser.addOption("Shelf-L1", CoralSystemPresets.L1_SCORE);
    chooser.addOption("Low-L2", CoralSystemPresets.L2);
    chooser.addOption("Mid-L3", CoralSystemPresets.L3);
    chooser.addOption("High-L4", CoralSystemPresets.L4);
    chooser.addOption("Algae-L1", CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_1);
    chooser.addOption("Algae-L2", CoralSystemPresets.PREPARE_DISLODGE_PART1_LEVEL_2);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  public CoralSystemPresets getSelected() {
    return chooser.getSelected();
  }
}
