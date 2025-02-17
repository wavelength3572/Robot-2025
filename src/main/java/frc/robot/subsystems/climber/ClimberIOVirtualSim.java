package frc.robot.subsystems.climber;

public class ClimberIOVirtualSim implements ClimberIO {

  private boolean climberDeployed = false; // Start without a coral

  private enum climberState {
    DEPLOYED,
    STOWED,
    CLIMB
  }

  private climberState currentIntakeState = climberState.STOWED;

  public ClimberIOVirtualSim() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberDeployed = climberDeployed;
  }

  @Override
  public void deployClimber() {
    climberDeployed = true;
  }

  @Override
  public void stowClimber() {
    climberDeployed = false;
  }

  @Override
  public boolean isClimberDeployed() {
    return climberDeployed;
  }
}
