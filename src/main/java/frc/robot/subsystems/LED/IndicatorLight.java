package frc.robot.subsystems.LED;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.IndicatorLightConstants.LED_EFFECTS;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.AlignmentUtils;
import java.util.Random;
import java.util.function.Supplier;

public class IndicatorLight extends SubsystemBase {

  private LED_EFFECTS currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
  private LED_EFFECTS LED_State = LED_EFFECTS.BLUEOMBRE;

  Supplier<CoralSystemPresets> getCurrentCoralPreset;
  Supplier<CoralSystemPresets> getSelectedCoralPreset;
  Supplier<CoralSystemPresets> getTargetCoralPreset;

  private CoralSystemPresets lastCurrentCoralPreset;
  private CoralSystemPresets lastSelectedCoralPreset;
  private CoralSystemPresets lastTargetCoralPreset;
  private CoralSystemPresets lastFinalLightingPreset =
      null; // Tracks last preset where final lighting applied

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
  private Supplier<Boolean> getIsCoralInRobot;
  private boolean pickupBlinkTriggered = false;

  public void setupLightingSuppliers(
      Supplier<CoralSystemPresets> currentPreset,
      Supplier<CoralSystemPresets> selectedPreset,
      Supplier<CoralSystemPresets> targetPreset,
      Supplier<Boolean> isCoralInRobot) {
    this.getCurrentCoralPreset = currentPreset;
    this.getSelectedCoralPreset = selectedPreset;
    this.getTargetCoralPreset = targetPreset;
    this.getIsCoralInRobot = isCoralInRobot;
  }

  public IndicatorLight() {

    lastSelectedCoralPreset = null;
    lastTargetCoralPreset = null;

    wlLED = new AddressableLED(IndicatorLightConstants.ADDRESSABLE_LED_PORT);
    wlLEDBuffer = new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    wlLED.setLength(wlLEDBuffer.getLength());
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

    updateElevatorLightingState();

    if (LED_State != LED_EFFECTS.BLINK) {
      LED_State = currentColor_GOAL;
    }
    switch (LED_State) {
        // Elevator-specific states: our updateElevatorLightingState() method has
        // already applied
        // the appropriate LED buffer. We simply do nothing here.
      case ELEVATOR_SELECTION_CHANGED, ELEVATOR_TARGET_CHANGED, ELEVATOR_CURRENT_CHANGED -> {
        // Elevator lighting is already updated.
      } // General states:
      case RED -> wlLED.setData(wlRedLEDBuffer);
      case YELLOW -> wlLED.setData(wlYellowLEDBuffer);
      case GREEN -> wlLED.setData(wlGreenLEDBuffer);
      case ORANGE -> wlLED.setData(wlOrangeLEDBuffer);
      case PURPLE -> wlLED.setData(wlPurpleLEDBuffer);
      case BLUE -> wlLED.setData(wlBlueLEDBuffer);
      case BLACK -> wlLED.setData(wlBlackLEDBuffer);
      case WHITE -> wlLED.setData(wlWhiteLEDBuffer);
      case RAINBOW -> doRainbow();
      case BLUEOMBRE -> doBlueOmbre();
      case BLINK -> blink();
      case BLINK_RED -> doBlinkRed();
      case BLINK_PURPLE -> blinkPurple();
      case PARTY -> party();
      case SEGMENTPARTY -> doSegmentParty();
      case EXPLOSION -> doExplosionEffect();
      case POLKADOT -> doPokadot();
      case SEARCH_LIGHT -> doSearchlightSingleEffect();
      case DRIVE_TO_REEF -> {} // TODO: Implement reef lighting logic

      default -> {}
    }

    // Publish the first 10 LEDs of wlLEDBuffer under "StripA" and the second 10
    // under "StripB"
    publishLEDsToDashboardFlipped("StripA", "StripB", wlLEDBuffer);
  }

  /**
   * Publishes the colors from an AddressableLEDBuffer as if it were two separate strips, flipping
   * each half so that LED 0 is at the bottom of each strip.
   *
   * @param baseKeyA NetworkTable key prefix for the first strip (e.g., "StripA")
   * @param baseKeyB NetworkTable key prefix for the second strip (e.g., "StripB")
   * @param buffer The AddressableLEDBuffer containing the full LED data
   */
  public void publishLEDsToDashboardFlipped(
      String baseKeyA, String baseKeyB, AddressableLEDBuffer buffer) {
    int length = buffer.getLength();
    int half = length / 2; // Split the buffer into two halves

    // 1) Publish the **first half**, but in **reverse order**
    for (int i = 0; i < half; i++) {
      int reversedIndex = half - 1 - i; // Flip the index
      var color8Bit = buffer.getLED8Bit(reversedIndex);
      String hex = String.format("#%02X%02X%02X", color8Bit.red, color8Bit.green, color8Bit.blue);
      SmartDashboard.putString(baseKeyA + "/led" + i, hex);
    }

    // 2) Publish the **second half**, but also in **reverse order**
    for (int i = 0; i < half; i++) {
      int reversedIndex = length - 1 - i; // Flip the index within the second half
      var color8Bit = buffer.getLED8Bit(reversedIndex);
      String hex = String.format("#%02X%02X%02X", color8Bit.red, color8Bit.green, color8Bit.blue);
      SmartDashboard.putString(baseKeyB + "/led" + i, hex);
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

    Color blinkColor = on ? Color.kWhite : Color.kBlack;
    for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
      wlLEDBuffer.setLED(i, blinkColor);
    }

    wlLED.setData(wlLEDBuffer);
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

  public void drivingToReefLighting() {
    currentColor_GOAL = LED_EFFECTS.DRIVE_TO_REEF;
  }

  public void doDrivingToReefLighting(
      Supplier<AlignmentUtils.ReefFaceSelection> reefFaceSelectionSupplier) {
    AlignmentUtils.ReefFaceSelection selection = reefFaceSelectionSupplier.get();

    if (selection == null || selection.getAcceptedFaceId() == null) {
      System.out.println("No valid reef face found. Cannot update LEDs.");
      return;
    }

    // Use the precomputed accepted distance (already calculated in drive logic)
    double distance = Units.metersToInches(selection.getAcceptedDistance()); // Convert to inches
    distance = Math.max(0, Math.min(distance, 125)); // Clamp between 0-125 inches

    // Determine LED brightness scale (closer = brighter)
    int maxBrightness = 255;
    int brightness = (int) (maxBrightness * (1 - (distance / 100.0)));

    // Determine LED color based on proximity
    Color targetColor;
    if (distance > 75) {
      targetColor = Color.kBlue; // Far away
    } else if (distance > 40) {
      targetColor = Color.kYellow; // Getting closer
    } else if (distance > 10) {
      targetColor = Color.kGreen; // Very close
    } else {
      targetColor =
          (Timer.getFPGATimestamp() % 0.5 < 0.25)
              ? Color.kGreen
              : Color.kBlack; // Flashing Green when at
      // target
    }

    // Apply gradient effect from center outward
    int center = wlLEDBuffer.getLength() / 2;
    for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
      int distanceFromCenter = Math.abs(center - i);
      int gradientFactor = 10; // Controls brightness falloff
      int ledBrightness = Math.max(0, brightness - (distanceFromCenter * gradientFactor));
      ledBrightness = Math.min(ledBrightness, maxBrightness);

      // Set LED with calculated color and brightness
      wlLEDBuffer.setLED(
          i,
          new Color(
              targetColor.red * (ledBrightness / 255.0),
              targetColor.green * (ledBrightness / 255.0),
              targetColor.blue * (ledBrightness / 255.0)));
    }

    // Apply LED updates
    wlLED.setData(wlLEDBuffer);
  }

  /**
   * Applies an elevator LED mapping effect using the provided preset and bright color. The mapping
   * is determined by the preset’s state: - L1_SCORE: LED 0 is bright. - L2_SCORE: LEDs 1-3 are
   * bright. - L3_SCORE: LEDs 4-6 are bright. - L4_SCORE: LEDs 7-9 are bright. The remainder of each
   * 10‑LED half is set to a muted version of the given color.
   *
   * @param preset The preset used to determine the LED mapping.
   * @param brightColor The color to use for the bright block.
   */
  private void setElevatorHeightEffect(CoralSystemPresets preset, Color brightColor) {

    final int totalLEDs = wlLEDBuffer.getLength(); // Expected 20
    final int halfLength = totalLEDs / 2; // 10 LEDs per strip

    int brightStart;
    int brightCount;
    switch (preset.getState()) {
      case STOW -> {
        currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
        return;
      }

      case L1_SCORE -> {
        brightStart = 0;
        brightCount = 1;
      }
      case L2_SCORE -> {
        brightStart = 1;
        brightCount = 3;
      }
      case L3_SCORE -> {
        brightStart = 4;
        brightCount = 3;
      }
      case L4_SCORE -> {
        brightStart = 7;
        brightCount = 3;
      }
      default -> {
        brightStart = 0;
        brightCount = 0;
      }
    }
    double mutedFactor = 0.3;
    Color mutedColor =
        new Color(
            brightColor.red * mutedFactor,
            brightColor.green * mutedFactor,
            brightColor.blue * mutedFactor);

    // For each half of the LED strip (assuming 20 LEDs split into two groups)
    for (int half = 0; half < 2; half++) {
      int offset = half * halfLength;
      // Fill the half with the muted color.
      for (int i = 0; i < halfLength; i++) {
        wlLEDBuffer.setLED(offset + i, mutedColor);
      }
      // Overwrite the designated block with the full bright color.
      for (int i = brightStart; i < brightStart + brightCount && i < halfLength; i++) {
        wlLEDBuffer.setLED(offset + i, brightColor);
      }
    }
    wlLED.setData(wlLEDBuffer);
  }

  /**
   * Checks the current elevator presets and updates the LED state if a change is detected. This
   * method sets LED_State to one of: - ELEVATOR_SELECTION_CHANGED (if the selected preset changed)
   * - ELEVATOR_TARGET_CHANGED (if the target preset changed) - ELEVATOR_CURRENT_CHANGED (if the
   * current preset now equals the target)
   */
  private void updateElevatorLightingState() {

    CoralSystemPresets currentPreset = getCurrentCoralPreset.get();
    CoralSystemPresets selectedPreset = getSelectedCoralPreset.get();
    CoralSystemPresets targetPreset = getTargetCoralPreset.get();

    if (currentPreset.equals(CoralSystemPresets.STARTUP)) {
      return;
    }

    // When the selected preset is PICKUP, decide which effect to use based on coral presence.
    if (selectedPreset != null && selectedPreset.equals(CoralSystemPresets.PICKUP)) {
      currentColor_GOAL = LED_EFFECTS.SEARCH_LIGHT;
      pickupBlinkTriggered = false;
      return;
    } else if (currentPreset != null && currentPreset.equals(CoralSystemPresets.PICKUP)) {
      // If a coral is present and we haven't triggered blink yet, do that.
      if (getIsCoralInRobot != null && getIsCoralInRobot.get()) {
        if (!pickupBlinkTriggered) {
          currentColor_GOAL = LED_EFFECTS.BLINK; // or a dedicated state like PICKUP_BLINK
          pickupBlinkTriggered = true;
        } else {
          currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
        }
      } else {
        currentColor_GOAL = LED_EFFECTS.SEARCH_LIGHT;
      }
      // Since we've handled PICKUP here, we might return.
      return;
    }

    // Detect change in the selected preset.
    if ((lastSelectedCoralPreset == null || !lastSelectedCoralPreset.equals(selectedPreset))
        && selectedPreset != null) {
      currentColor_GOAL = LED_EFFECTS.ELEVATOR_SELECTION_CHANGED;
      updateSelectedLighting(selectedPreset);
      lastSelectedCoralPreset = selectedPreset;
    }
    // Detect change in the target preset.
    if (lastTargetCoralPreset == null || !lastTargetCoralPreset.equals(targetPreset)) {
      currentColor_GOAL = LED_EFFECTS.ELEVATOR_TARGET_CHANGED;
      updateTransitionLighting(targetPreset);
      lastTargetCoralPreset = targetPreset;
    }
    // If the current preset now equals the target and it’s a change from the last
    // final state,
    // update final lighting.
    if (currentPreset.equals(targetPreset)
        && (lastFinalLightingPreset == null || !lastFinalLightingPreset.equals(targetPreset))) {
      currentColor_GOAL = LED_EFFECTS.ELEVATOR_CURRENT_CHANGED;
      updateFinalLighting(targetPreset);
      lastFinalLightingPreset = targetPreset;
    }
  }

  private void updateSelectedLighting(CoralSystemPresets selectedPreset) {
    System.out.println("Elevator selection changed: " + selectedPreset);
    // Use yellow for selection preview.
    setElevatorHeightEffect(selectedPreset, Color.kYellow);
  }

  private void updateTransitionLighting(CoralSystemPresets targetPreset) {
    System.out.println("Elevator target changed: " + targetPreset);
    // Use blue for transition.
    setElevatorHeightEffect(targetPreset, Color.kYellow);
  }

  private void updateFinalLighting(CoralSystemPresets targetPreset) {
    System.out.println("Elevator current reached target: " + targetPreset);
    // Use green to indicate the final state.
    setElevatorHeightEffect(targetPreset, Color.kGreen);
  }

  public void doSearchlightSingleEffect() {
    // Only update after the desired update interval.
    if (effectTimer.get() < updateInterval) {
      return;
    }

    // Clear the LED buffer (set all LEDs to off/black).
    for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
      wlLEDBuffer.setLED(i, Color.kBlack);
    }

    // Determine the half-length. For 20 LEDs, half will be 10.
    int half = wlLEDBuffer.getLength() / 2;

    // Compute an effective counter that advances 5x slower.
    int effectiveCounter = counter / 5;

    // Compute the position for the searchlight within [0, half-1]
    // using a ping-pong (bounce) function so that it moves from 0 to half-1 and
    // back.
    int range = half - 1;
    int pos = effectiveCounter % (2 * range);
    if (pos > range) {
      pos = 2 * range - pos;
    }

    // Decide the searchlight color based on your current logic:
    // If the current preset equals PICKUP, use white; otherwise use yellow.
    Color searchlightColor;
    CoralSystemPresets currentPreset = getCurrentCoralPreset.get();
    if (currentPreset.equals(CoralSystemPresets.PICKUP)) {
      searchlightColor = Color.kWhite;
    } else {
      searchlightColor = Color.kYellow;
    }

    // Set the LED at position 'pos' in the first half and at 'pos' in the second
    // half.
    wlLEDBuffer.setLED(pos, searchlightColor);
    wlLEDBuffer.setLED(half + pos, searchlightColor);

    // Send the updated buffer to the LED hardware.
    wlLED.setData(wlLEDBuffer);

    // Increment the counter and reset the timer.
    counter++;
    effectTimer.reset();
  }
}
