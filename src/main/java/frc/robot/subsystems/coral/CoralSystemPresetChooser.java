package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSystemPresetChooser {
  private SendableChooser<CoralSystemPresets> chooser = new SendableChooser<>();
  private boolean hasReachedStow = false;
  private final CoralSystem coralSystem; // Reference to your subsystem

  public CoralSystemPresetChooser(CoralSystem coralSystem) {
    this.coralSystem = coralSystem;
    resetToStartupState();
  }

  /** Resets chooser to startup state where only "Stow" is available. */
  private void resetToStartupState() {
    chooser = new SendableChooser<>(); // Create a new instance to clear previous options
    chooser.setDefaultOption("Stow", CoralSystemPresets.STOW);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  /** Checks if the system has physically reached "Stow" and updates chooser accordingly. */
  public void checkAndUpdate() {
    if (!hasReachedStow && coralSystem.isAtStowPosition()) { // Check physical position
      hasReachedStow = true;
      enableAllOptions();
    }
  }

  /** Enables all preset options once the robot reaches "Stow." */
  private void enableAllOptions() {
    chooser = new SendableChooser<>(); // Create a new chooser to clear old options
    chooser.setDefaultOption("Stow", CoralSystemPresets.STOW);
    chooser.addOption("Pickup", CoralSystemPresets.PICKUP);
    chooser.addOption("Score Level 1", CoralSystemPresets.SCORE_LEVEL_1);
    chooser.addOption("Score Level 2", CoralSystemPresets.SCORE_LEVEL_2);
    chooser.addOption("Score Level 3", CoralSystemPresets.SCORE_LEVEL_3);
    chooser.addOption("Score Level 4", CoralSystemPresets.SCORE_LEVEL_4);
    chooser.addOption("Dislodge Level 1", CoralSystemPresets.DISLODGE_LEVEL_1);
    chooser.addOption("Dislodge Level 2", CoralSystemPresets.DISLODGE_LEVEL_2);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  /** Returns the currently selected CoralSystemPreset. */
  public CoralSystemPresets getSelected() {
    return chooser.getSelected();
  }
}
