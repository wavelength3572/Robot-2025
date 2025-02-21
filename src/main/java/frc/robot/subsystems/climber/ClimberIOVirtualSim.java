package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;

public class ClimberIOVirtualSim implements ClimberIO {

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;

  public ClimberIOVirtualSim() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.currentClimbState = currentClimberState;
  }

  @Override
  public void deployClimber() {
    currentClimberState = CLIMB_STATE.DEPLOY;
  }

  @Override
  public void stowClimber() {
    currentClimberState = CLIMB_STATE.STOWED;
  }

  @Override
  public boolean isClimberDeployed() {
    return (this.currentClimberState != CLIMB_STATE.STOWED);
  }
}
