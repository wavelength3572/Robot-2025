package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSystemPresetChooser {
  private SendableChooser<CoralSystemPresets> chooser = new SendableChooser<>();
  private boolean hasReachedStow = false;
  private final CoralSystem coralSystem;

  public CoralSystemPresetChooser(CoralSystem coralSystem) {
    this.coralSystem = coralSystem;
    resetToStartupState();
  }

  /** Resets chooser to startup state where only "Stow" is available. */
  private void resetToStartupState() {
    chooser = new SendableChooser<>(); // Clear previous options
    chooser.setDefaultOption("Stow", CoralSystemPresets.STOW);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  /** Checks if the system has physically reached "Stow" and updates chooser accordingly. */
  public void checkAndUpdate() {
    if (!hasReachedStow && coralSystem.getCurrentCoralPreset() == CoralSystemPresets.STOW) {
      hasReachedStow = true;
      enableAllOptions();
    } else if (hasReachedStow && coralSystem.getCurrentCoralPreset() != CoralSystemPresets.STOW) {
      // If we move away from Stow, reset back to only allowing Stow
      hasReachedStow = false;
      resetToStartupState();
    }
  }

  /** Enables all preset options once the robot reaches "Stow." */
  private void enableAllOptions() {
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Stow", CoralSystemPresets.STOW);
    chooser.addOption("Pickup", CoralSystemPresets.PICKUP);
    chooser.addOption("L1", CoralSystemPresets.SCORE_LEVEL_1);
    chooser.addOption("L2", CoralSystemPresets.SCORE_LEVEL_2);
    chooser.addOption("L3", CoralSystemPresets.SCORE_LEVEL_3);
    chooser.addOption("L4", CoralSystemPresets.SCORE_LEVEL_4);
    chooser.addOption("Dislodge L1", CoralSystemPresets.PREPARE_DISLODGE_LEVEL_1);
    chooser.addOption("Dislodge L2", CoralSystemPresets.PREPARE_DISLODGE_LEVEL_2);
    SmartDashboard.putData("Coral Position Chooser", chooser);
  }

  /** Returns the currently selected CoralSystemPreset. */
  public CoralSystemPresets getSelected() {
    return chooser.getSelected();
  }
}
