package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED.IndicatorLight;
import frc.robot.subsystems.LED.IndicatorLightConstants.LED_EFFECTS;

public final class IndicatorLightCommands {

  // Private constructor prevents instantiation.
  private IndicatorLightCommands() {}

  /**
   * Returns a command that sets the indicator light to blink mode.
   *
   * @param indicatorLight The IndicatorLight subsystem instance.
   * @return The command.
   */
  public static Command blink(IndicatorLight indicatorLight) {
    return new InstantCommand(() -> indicatorLight.blink(), indicatorLight);
  }

  /**
   * Returns a command that sets the indicator light to a solid red color.
   *
   * @param indicatorLight The IndicatorLight subsystem instance.
   * @return The command.
   */
  public static Command setRed(IndicatorLight indicatorLight) {
    return new InstantCommand(() -> indicatorLight.red(), indicatorLight);
  }

  /**
   * Returns a command that sets the indicator light to a solid blue color.
   *
   * @param indicatorLight The IndicatorLight subsystem instance.
   * @return The command.
   */
  public static Command setBlue(IndicatorLight indicatorLight) {
    return new InstantCommand(() -> indicatorLight.blue(), indicatorLight);
  }

  /**
   * Returns a command that sets the indicator light mode based on the given LED effect.
   *
   * @param indicatorLight The IndicatorLight subsystem instance.
   * @param mode The LED effect mode.
   * @return The command.
   */
  public static Command setMode(IndicatorLight indicatorLight, LED_EFFECTS mode) {
    return new InstantCommand(
        () -> {
          // Assuming you have methods or a setter that applies the effect,
          // e.g. indicatorLight.setCurrentEffect(mode);
          switch (mode) {
            case RED:
              indicatorLight.red();
              break;
            case BLUE:
              indicatorLight.blue();
              break;
            case GREEN:
              indicatorLight.green();
              break;
            case RAINBOW:
              indicatorLight.rainbow();
              break;
            case BLINK:
              indicatorLight.blink();
              break;
              // Add other cases as needed.
            default:
              // Fallback or no-op.
              break;
          }
        },
        indicatorLight);
  }
}
