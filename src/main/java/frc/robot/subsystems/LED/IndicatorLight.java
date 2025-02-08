package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.IndicatorLightConstants.LED_EFFECTS;
import java.util.Random;

public class IndicatorLight extends SubsystemBase {

  private LED_EFFECTS currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
  private LED_EFFECTS LED_State = LED_EFFECTS.BLUEOMBRE;

  private AddressableLED wlLED;
  private AddressableLEDBuffer wlLEDBuffer;
  private AddressableLEDBuffer wlGreenLEDBuffer;
  private AddressableLEDBuffer wlOrangeLEDBuffer;
  private AddressableLEDBuffer wlPurpleLEDBuffer;
  private AddressableLEDBuffer wlRedLEDBuffer;
  private AddressableLEDBuffer wlYellowLEDBuffer;
  private AddressableLEDBuffer wlBlueLEDBuffer;
  private AddressableLEDBuffer wlIndigoLEDBuffer;
  private AddressableLEDBuffer wlVioletLEDBuffer;
  private AddressableLEDBuffer wlWhiteLEDBuffer;
  private AddressableLEDBuffer wlBlackLEDBuffer;

  private AddressableLEDSim wlLEDSim;

  // Store what the last hue of the first pixel is
  private int rainbowFirstPixelHue = 0;
  private int currentSaturation = 100;
  private boolean forward = true;
  private int counter = 0;
  private double lastTime = 0.0;
  private double blinkTime = 0.0;
  private boolean on = false;
  private int skittleCount = 0;

  private Random random = new Random();

  private Timer effectTimer = new Timer();
  private final double restartInterval = 5.0; // Restart effect every 10 seconds
  private int effectPhase = 0;
  private final int maxBrightness = 255;
  private int center = 9;
  private final double updateInterval = 0.05; // Interval in seconds for updates

  // Current character index to display
  private int characterIndex = 0;

  // Interval in seconds to wait before switching to the next character
  private static final double CHARACTER_DISPLAY_INTERVAL = 2.0;

  public IndicatorLight() {
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    wlLED = new AddressableLED(IndicatorLightConstants.ADDRESSABLE_LED_PORT);
    wlLEDSim = new AddressableLEDSim();
    // Length is expensive to set, so only set it once, then just update data
    wlLEDBuffer = new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    wlLED.setLength(wlLEDBuffer.getLength());
    wlLEDSim.setLength(wlLEDBuffer.getLength());
    center = wlLEDBuffer.getLength() / 2;
    wlLED.setData(wlLEDBuffer);
    wlLED.start();

    wlGreenLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlGreenLEDBuffer.getLength(); i++) {
      wlGreenLEDBuffer.setHSV(i, IndicatorLightConstants.GREEN_HUE, 255, 128);
    }

    wlOrangeLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlOrangeLEDBuffer.getLength(); i++) {
      wlOrangeLEDBuffer.setHSV(i, IndicatorLightConstants.ORANGE_HUE, 255, 128);
    }

    wlPurpleLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlPurpleLEDBuffer.getLength(); i++) {
      wlPurpleLEDBuffer.setHSV(i, IndicatorLightConstants.PURPLE_HUE, 63, 92);
    }

    wlRedLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlRedLEDBuffer.getLength(); i++) {
      wlRedLEDBuffer.setHSV(i, IndicatorLightConstants.RED_HUE, 255, 128);
    }

    wlYellowLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlYellowLEDBuffer.getLength(); i++) {
      wlYellowLEDBuffer.setHSV(i, IndicatorLightConstants.YELLOW_HUE, 255, 128);
    }

    wlBlueLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlBlueLEDBuffer.getLength(); i++) {
      wlBlueLEDBuffer.setLED(i, Color.kBlue);
    }

    wlIndigoLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlIndigoLEDBuffer.getLength(); i++) {
      wlIndigoLEDBuffer.setLED(i, Color.kIndigo);
    }

    wlVioletLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlVioletLEDBuffer.getLength(); i++) {
      wlVioletLEDBuffer.setLED(i, Color.kViolet);
    }

    wlWhiteLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlWhiteLEDBuffer.getLength(); i++) {
      wlWhiteLEDBuffer.setLED(i, Color.kWhite);
    }

    wlBlackLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlBlackLEDBuffer.getLength(); i++) {
      wlBlackLEDBuffer.setLED(i, Color.kBlack);
    }

    effectTimer.start();
  }

  @Override
  public void periodic() {
    if (LED_State != LED_EFFECTS.BLINK) {
      LED_State = currentColor_GOAL;
    }
    switch (LED_State) {
      case RED:
        wlLED.setData(wlRedLEDBuffer);
        break;
      case YELLOW:
        wlLED.setData(wlYellowLEDBuffer);
        break;
      case GREEN:
        wlLED.setData(wlGreenLEDBuffer);
        break;
      case ORANGE:
        wlLED.setData(wlOrangeLEDBuffer);
        break;
      case PURPLE:
        wlLED.setData(wlPurpleLEDBuffer);
        break;
      case BLUE:
        wlLED.setData(wlBlueLEDBuffer);
        break;
      case BLACK:
        wlLED.setData(wlBlackLEDBuffer);
        break;
      case WHITE:
        wlLED.setData(wlWhiteLEDBuffer);
        break;
      case RAINBOW:
        doRainbow();
        break;
      case BLUEOMBRE:
        doBlueOmbre();
        break;
      case BLINK:
        blink();
        break;
      case BLINK_RED:
        doBlinkRed();
        break;
      case BLINK_PURPLE:
        blinkPurple();
        break;
      case PARTY:
        party();
        break;
      case SEGMENTPARTY:
        doSegmentParty();
        break;
      case EXPLOSION:
        doExplosionEffect();
        break;
      case CHARACTERS:
        doCycleCharacters();
        break;
      case POLKADOT:
        doPokadot();
        break;
      case SEARCH_LIGHT:
        doSearchlightEffect();
        break;
        // case DRIVE_TO_AMP:
        //   doDrivingToAmpLighting();
        //   break;
      default:
        break;
    }
  }

  public void blueOmbre() {
    currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
  }

  public void rainbow() {
    currentColor_GOAL = LED_EFFECTS.RAINBOW;
  }

  public void party() {
    currentColor_GOAL = LED_EFFECTS.PARTY;
  }

  public void segmentParty() {
    currentColor_GOAL = LED_EFFECTS.SEGMENTPARTY;
  }

  public void explosion() {
    currentColor_GOAL = LED_EFFECTS.EXPLOSION;
  }

  public void polkadot() {
    currentColor_GOAL = LED_EFFECTS.POLKADOT;
  }

  public void doExplosionEffect() {
    double elapsedTime = effectTimer.get();

    // Automatically restart the effect after a specific interval
    if (elapsedTime > restartInterval) {
      effectPhase = 1; // Reset to start phase
      effectTimer.reset();
    }

    // Determine the update step based on the elapsed time
    int step = (int) (elapsedTime / updateInterval);

    if (effectPhase == 1) {
      // Expansion phase
      if (step <= center) {
        for (int i = 0; i <= step; i++) {
          int brightness = Math.max(0, maxBrightness - ((maxBrightness / center) * i));
          wlLEDBuffer.setRGB(center + i, brightness, brightness, 0);
          wlLEDBuffer.setRGB(center - i, brightness, brightness, 0);
        }
        wlLED.setData(wlLEDBuffer);
      } else {
        effectPhase = 2; // Move to fading phase
      }
    } else if (effectPhase == 2) {
      // Fading phase
      int fadeStep = maxBrightness - (int) (step * 5.0 / updateInterval);
      if (fadeStep > 0) {
        for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
          int distance = Math.abs(center - i);
          int brightness = Math.max(0, fadeStep - ((maxBrightness / center) * distance));
          wlLEDBuffer.setRGB(i, brightness, brightness, 0);
        }
        wlLED.setData(wlLEDBuffer);
      } else {
        effectPhase = 0; // End the effect and wait for the next restart
      }
    }
  }

  public void doParty() {
    // For every pixel
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value
      int red = random.nextInt(256);
      int green = random.nextInt(256);
      int blue = random.nextInt(256);
      wlLEDBuffer.setRGB(i, red, green, blue);
    }
    // Set the LEDs
    wlLED.setData(wlLEDBuffer);
  }

  private void doSegmentParty() {
    // First, turn off all LEDs
    // for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
    // wlLEDBuffer.setRGB(i, 255,255, 255);
    // }
    if (counter > IndicatorLightConstants.UPDATE_FREQUENCY) {
      counter = 0;

      // Decide how many segments to create
      int numberOfSegments = 1 + random.nextInt(10); // For example, 1 to 3 segments

      for (int segment = 0; segment < numberOfSegments; segment++) {
        // Select a random color from the palette
        int[] color =
            IndicatorLightConstants.colorPalette[
                random.nextInt(IndicatorLightConstants.colorPalette.length)];

        // Choose random start point and length
        int start = random.nextInt(wlLEDBuffer.getLength());
        int length = 1 + random.nextInt(wlLEDBuffer.getLength() - start); // Ensure segment fits

        // Set the colors
        for (int i = start; i < start + length; i++) {
          wlLEDBuffer.setRGB(i, color[0], color[1], color[2]);
        }
      }
    } else counter++;

    // Update the LED strip
    wlLED.setData(wlLEDBuffer);
  }

  private void doPokadot() {
    // For every pixel
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value
      int red = random.nextInt(256);
      int green = random.nextInt(256);
      int blue = random.nextInt(256);
      wlLEDBuffer.setRGB(i, red, green, blue);
    }
    // Set the LEDs
    wlLED.setData(wlLEDBuffer);
  }

  public void doRainbow() {
    // For every pixel
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / wlLEDBuffer.getLength())) % 180;
      // Set the value
      wlLEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    // Set the LEDs
    wlLED.setData(wlLEDBuffer);
  }

  public void doBlueOmbre() {
    // For every pixel
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var saturation = (currentSaturation + (i * 255 / wlLEDBuffer.getLength())) % 255;
      // Set the value
      wlLEDBuffer.setHSV(i, 103, 255, saturation);
    }

    // Update currentSaturation and ensure it stays within the 0-255 range
    if (forward) {
      // Move the saturation up
      currentSaturation += 3;
      if (currentSaturation >= 255) {
        currentSaturation = 255;
        forward = false;
      }
    } else {
      // Move the saturation down
      currentSaturation -= 3;
      if (currentSaturation <= 0) {
        currentSaturation = 0;
        forward = true;
      }
    }

    // Set the LEDs
    wlLED.setData(wlLEDBuffer);
  }

  public void blink() {
    LED_State = LED_EFFECTS.BLINK;
    double timeStamp = Timer.getFPGATimestamp();
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.05) {
      on = !on;
      lastTime = timeStamp;
    }
    if (timeStamp - blinkTime > 1.0) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }
    if (on) {
      wlLED.setData(wlWhiteLEDBuffer);
    } else {
      wlLED.setData(wlBlackLEDBuffer);
    }
  }

  public void blinkRed() {
    LED_State = LED_EFFECTS.BLINK_RED;
  }

  public void doBlinkRed() {
    double timeStamp = Timer.getFPGATimestamp();
    if (timeStamp - lastTime > 0.1) {
      on = !on;
      lastTime = timeStamp;
    }
    if (on) {
      wlLED.setData(wlRedLEDBuffer);
    } else {
      wlLED.setData(wlBlackLEDBuffer);
    }
  }

  public void rainbowBlink() {
    LED_State = LED_EFFECTS.BLINK;
    double timeStamp = Timer.getFPGATimestamp();
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.1) {
      on = !on;
      lastTime = timeStamp;
      skittleCount++;
      skittleCount = skittleCount % 7;
      if (skittleCount == 0) wlLED.setData(wlRedLEDBuffer);
      if (skittleCount == 1) wlLED.setData(wlOrangeLEDBuffer);
      if (skittleCount == 2) wlLED.setData(wlYellowLEDBuffer);
      if (skittleCount == 3) wlLED.setData(wlGreenLEDBuffer);
      if (skittleCount == 4) wlLED.setData(wlBlueLEDBuffer);
      if (skittleCount == 5) wlLED.setData(wlIndigoLEDBuffer);
      if (skittleCount == 6) wlLED.setData(wlVioletLEDBuffer);
    }
    if (timeStamp - blinkTime > 1.5) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }
  }

  public void blinkPurple() {
    LED_State = LED_EFFECTS.BLINK_PURPLE;
    double timeStamp = Timer.getFPGATimestamp();
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.05) {
      on = !on;
      lastTime = timeStamp;
    }
    if (timeStamp - blinkTime > 1.0) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }
    if (on) {
      wlLED.setData(wlWhiteLEDBuffer);
    } else {
      wlLED.setData(wlPurpleLEDBuffer);
    }
  }

  public void green() {
    currentColor_GOAL = LED_EFFECTS.GREEN;
  }

  public void orange() {
    currentColor_GOAL = LED_EFFECTS.ORANGE;
  }

  public void purple() {
    currentColor_GOAL = LED_EFFECTS.PURPLE;
  }

  public void red() {
    currentColor_GOAL = LED_EFFECTS.RED;
  }

  public void yellow() {
    currentColor_GOAL = LED_EFFECTS.YELLOW;
  }

  public void blue() {
    currentColor_GOAL = LED_EFFECTS.BLUE;
  }

  // Generic method to set character colors on the LED strip
  public void setCharacterColors(Color[] colors, int[] ledsPerColor) {
    int ledIndex = 0;
    for (int colorIndex = 0; colorIndex < colors.length; colorIndex++) {
      Color color = colors[colorIndex];
      for (int i = 0; i < ledsPerColor[colorIndex]; i++) {
        // Check to prevent overflow if the total LEDs exceed the strip length
        if (ledIndex < wlLEDBuffer.getLength()) {
          wlLEDBuffer.setLED(ledIndex++, color);
        }
      }
    }
    // Fill the rest with black (off) if there are any remaining LEDs
    while (ledIndex < wlLEDBuffer.getLength()) {
      wlLEDBuffer.setLED(ledIndex++, Color.kBlack);
    }
    wlLED.setData(wlLEDBuffer);
  }

  // Method to update character display based on the timer
  public void doCycleCharacters() {
    if (effectTimer.get() >= CHARACTER_DISPLAY_INTERVAL) {
      setCharacterColors(
          CharacterLights.ALL_CHARACTERS_COLORS[characterIndex],
          CharacterLights.ALL_CHARACTERS_LEDS_PER_COLOR[characterIndex]);

      // Move to the next character, looping back if necessary
      characterIndex = (characterIndex + 1) % CharacterLights.ALL_CHARACTERS_COLORS.length;

      // Reset the timer to count the interval for the next character
      effectTimer.reset();
    }
  }

  public void CycleCharacters() {
    currentColor_GOAL = LED_EFFECTS.CHARACTERS;
  }

  public void doSearchlightEffect() {
    int searchlightSize = 3; // The height of the searchlight beam
    int maxPosition =
        wlLEDBuffer.getLength()
            - searchlightSize; // Maximum position for the start of the searchlight
    final int searchlightBrightness = 255; // Max brightness for the searchlight
    final int backgroundBrightness = 0; // Background brightness

    // Update the effect every few iterations to make the movement visible
    if (effectTimer.get() >= updateInterval) {
      // Clear the previous state
      for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
        wlLEDBuffer.setHSV(i, 0, 0, backgroundBrightness); // Example with yellow color, dimmed out
      }

      // Calculate the position of the searchlight
      int startPosition = counter % (maxPosition * 2);
      if (startPosition > maxPosition) {
        // Reverse direction
        startPosition = maxPosition - (startPosition - maxPosition);
      }

      // Set the searchlight pixels
      for (int i = startPosition;
          i < startPosition + searchlightSize && i < wlLEDBuffer.getLength();
          i++) {
        wlLEDBuffer.setHSV(
            i, 0, 0, searchlightBrightness); // Example with yellow color, full brightness
      }

      // Update the LEDs
      wlLED.setData(wlLEDBuffer);

      // Increment the counter for the next position
      counter++;

      // Reset the timer for the next update
      effectTimer.reset();
    }
  }

  public void searchlight() {
    currentColor_GOAL =
        LED_EFFECTS.SEARCH_LIGHT; // Assuming CUSTOM is a placeholder for new effects
  }

  public void drivingToAmpLihgting() {
    currentColor_GOAL = LED_EFFECTS.DRIVE_TO_AMP;
  }

  // public void doDrivingToAmpLighting() {
  //   double distance =
  //       307.4
  //           - Units.metersToInches(
  //               RobotContainer.getInstance()
  //                   .getDrive()
  //                   .getPose()
  //                   .getY()); // Get the current distance to the AprilTag
  //   if (distance > 0) {

  //     // Ensure distance is within bounds
  //     distance = Math.max(0, Math.min(distance, 125)); // Clamps distance between 0 and 100
  // inches

  //     // Calculate overall brightness based on the inverse of the distance
  //     int maxBrightness = 255; // Max LED brightness
  //     // Inverse proportionality: at 0 inches, brightness is max; at >100 inches, brightness is
  // min
  //     // (0).
  //     int brightness = (int) (maxBrightness * (1 - (distance / 100)));

  //     for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
  //       int center = wlLEDBuffer.getLength() / 2;
  //       int distanceFromCenter = Math.abs(center - i);

  //       // Calculate brightness for each LED
  //       // This part ensures a smoother gradient by decreasing brightness linearly from the
  // center
  //       // to the edges
  //       // Adjust 'gradientFactor' to control the spread of the max brightness area
  //       int gradientFactor = 10; // Control how quickly the brightness falls off from the center
  //       int ledBrightness = Math.max(0, brightness - (distanceFromCenter * gradientFactor));

  //       // Ensure ledBrightness does not exceed maxBrightness
  //       ledBrightness = Math.min(ledBrightness, maxBrightness);

  //       // Set the LED to white with the calculated brightness
  //       wlLEDBuffer.setHSV(i, 0, 0, ledBrightness);
  //     }

  //     // Apply the updated LED buffer to the strip
  //     wlLED.setData(wlLEDBuffer);
  //   }
  // }

  private int getAmpTagID() {
    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      return 6;
    } else {
      return 5;
    }
  }
}
