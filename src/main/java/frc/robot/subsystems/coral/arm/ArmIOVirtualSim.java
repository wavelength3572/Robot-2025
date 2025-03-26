package frc.robot.subsystems.coral.arm;

import edu.wpi.first.wpilibj.Timer;

public class ArmIOVirtualSim implements ArmIO {
  // Current target in degrees and corresponding encoder rotations.
  private double armTargetDEG = ArmConstants.armStartAngle;
  private double armTargetEncoderRotations =
      ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  // Virtual encoder position (in rotations) used for simulation.
  private double armVirtualEncoder = ArmConstants.armStartAngle * ArmConstants.kArmGearing / 360.0;

  // Simple P-based simulation constants
  private static final double kP_DEFAULT = 15.0; // Proportional gain
  private static final double MAX_ROT_PER_SEC = 35.0; // Hard limit on rotational velocity

  // We'll track time so movement is dt-based rather than a fixed step
  private double lastUpdateTime;

  public ArmIOVirtualSim() {
    // Initialize time for our simulation
    lastUpdateTime = Timer.getFPGATimestamp();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // 1) Calculate dt
    double now = Timer.getFPGATimestamp();
    double dt = now - lastUpdateTime;
    lastUpdateTime = now;

    // If dt is too small or negative, default to ~20 ms
    if (dt < 1.0e-6) {
      dt = 0.02;
    }

    // 2) Fill the ArmIOInputs (for logging, LiveWindow, etc.)
    inputs.targetAngleDEG = armTargetDEG;
    inputs.currentAngleDEG = armVirtualEncoder * 360.0 / ArmConstants.kArmGearing;
    inputs.targetEncoderRotations = armTargetEncoderRotations;
    inputs.encoderRotations = armVirtualEncoder;

    // Not modeling actual amps or feedforward in this simple sim
    inputs.currentAmps = 0.0;
    inputs.armArbFF = 0.0;
    inputs.armArbFF_COS = 0.0;
    inputs.TBE_Valid = true;

    // 3) Compute a velocity based on proportional error, clamp it
    double errorRotations = (armTargetEncoderRotations - armVirtualEncoder);
    double velocityRotPerSec = kP_DEFAULT * errorRotations;

    // Clamp velocity to +/- MAX_ROT_PER_SEC
    if (velocityRotPerSec > MAX_ROT_PER_SEC) velocityRotPerSec = MAX_ROT_PER_SEC;
    if (velocityRotPerSec < -MAX_ROT_PER_SEC) velocityRotPerSec = -MAX_ROT_PER_SEC;

    // 4) Update the virtual position using v * dt
    armVirtualEncoder += velocityRotPerSec * dt;

    // For logging, record the velocity in RPM if desired
    inputs.velocityRPM = velocityRotPerSec * 60.0; // rotations/sec -> rotations/min
  }

  @Override
  public void setTargetAngleDEG(double requestedPosition) {
    this.armTargetDEG = requestedPosition;
    this.armTargetEncoderRotations = requestedPosition * ArmConstants.kArmGearing / 360.0;
  }

  @Override
  public double getTargetAngleDEG() {
    return this.armTargetDEG;
  }

  @Override
  public double getCurrentArmDEG() {
    return this.armVirtualEncoder * 360.0 / ArmConstants.kArmGearing;
  }

  @Override
  public void setInitialAngle(double initialDegree) {
    // In simulation, we don't have a real sensor, so let's forcibly set everything.
    initialDegree = 110; // If you want a different start angle, just change this line.

    armVirtualEncoder = initialDegree * ArmConstants.kArmGearing / 360.0;
    armTargetDEG = initialDegree;
    armTargetEncoderRotations = armVirtualEncoder;
  }
}
