package frc.robot.subsystems.LED;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.IndicatorLightConstants.LED_EFFECTS;
import frc.robot.subsystems.coral.CoralSystemPresets;
import frc.robot.util.AlignmentUtils;
import frc.robot.util.BranchAlignmentUtils;
import frc.robot.util.BranchAlignmentUtils.BranchAlignmentStatus;
import frc.robot.util.RobotStatus;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IndicatorLight extends SubsystemBase {

  private LED_EFFECTS currentColor_GOAL = LED_EFFECTS.BLACK;
  private LED_EFFECTS LED_State = LED_EFFECTS.BLACK;

  // Define constants for the blink period (in seconds)
  private static final double MAX_BLINK_PERIOD = 0.5; // far from target, slow blink
  private static final double MIN_BLINK_PERIOD = 0.001; // very close, fast blink

  public static final double MIN_TOLERANCE_BLINK = 0.025; // below this, blink is at min period
  public static final double MAX_TOLERANCE_BLINK = 0.150; // above this, blink is at max period

  // Variable to hold the current blink period computed from the error
  private double currentBlinkPeriod = MAX_BLINK_PERIOD;

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
  private Supplier<Boolean> getHaveCoral;
  private boolean pickupBlinkTriggered = false;

  private boolean branchAlignmentOn = true; // start with branchAlignment lighting on

  private AddressableLEDBuffer currentActiveBuffer;

  public void setupLightingSuppliers(
      Supplier<CoralSystemPresets> currentPreset,
      Supplier<CoralSystemPresets> selectedPreset,
      Supplier<CoralSystemPresets> targetPreset,
      Supplier<Boolean> getHaveCoral) {
    this.getCurrentCoralPreset = currentPreset;
    this.getSelectedCoralPreset = selectedPreset;
    this.getTargetCoralPreset = targetPreset;
    this.getHaveCoral = getHaveCoral;
  }

  public IndicatorLight() {

    lastSelectedCoralPreset = null;
    lastTargetCoralPreset = null;

    wlLED = new AddressableLED(IndicatorLightConstants.ADDRESSABLE_LED_PORT);
    wlLEDBuffer = new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    wlLED.setLength(wlLEDBuffer.getLength());
    center = wlLEDBuffer.getLength() / 2;
    currentActiveBuffer = wlLEDBuffer; // Set the default active buffer
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
    for (int i = 0; i < wlYellowLEDBuffer.getLength(); i++) {
      wlYellowLEDBuffer.setLED(i, Color.kYellow);
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
    currentColor_GOAL = updateLightingGoal();

    updateBranchAlignmentLighting(); // override with branch alignment lighting if close enough

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
      case RED -> setActiveBuffer(wlRedLEDBuffer);
      case YELLOW -> setActiveBuffer(wlYellowLEDBuffer);
      case GREEN -> setActiveBuffer(wlGreenLEDBuffer);
      case ORANGE -> setActiveBuffer(wlOrangeLEDBuffer);
      case PURPLE -> setActiveBuffer(wlPurpleLEDBuffer);
      case BLUE -> setActiveBuffer(wlBlueLEDBuffer);
      case BLACK -> setActiveBuffer(wlBlackLEDBuffer);
      case WHITE -> setActiveBuffer(wlWhiteLEDBuffer);
      case RAINBOW -> doRainbow();
      case BLUEOMBRE -> doBlueOmbre();
      case BLINK -> doBlink();
      case BLINK_RED -> doBlinkRed();
      case BLINK_PURPLE -> blinkPurple();
      case PARTY -> party();
      case SEGMENTPARTY -> doSegmentParty();
      case EXPLOSION -> doExplosionEffect();
      case POLKADOT -> doPokadot();
      case SEARCH_LIGHT -> doSearchlightSingleEffect();
      case DYNAMIC_BLINK -> dynamicBlink();
      case DRIVE_TO_REEF -> {} // TODO:
        // Implement
        // reef
        // lighting
        // logic

      default -> {}
    }

    // Publish the first 10 LEDs of wlLEDBuffer under "StripA" and the second 10
    // under "StripB"
    publishLEDsToDashboardFlipped("StripA", currentActiveBuffer);
  }

  private void updateBranchAlignmentLighting() {

    double lateralError = BranchAlignmentUtils.getLateralErrorToNearestPole();
    updateBlinkPeriod(lateralError);

    if (branchAlignmentOn && RobotStatus.haveCoral()) {
      BranchAlignmentStatus state = BranchAlignmentUtils.getCurrentBranchAlignmentStatus();
      switch (state) {
        case GREEN:
          if (lateralError <= BranchAlignmentUtils.LATERAL_THRESHOLD_SOLID_GREEN) {
            currentColor_GOAL = LED_EFFECTS.GREEN;
          } else {
            currentColor_GOAL = LED_EFFECTS.DYNAMIC_BLINK;
          }
          break;
        case RED:
          currentColor_GOAL = LED_EFFECTS.RED;
          break;
        case NONE:
        default:
          break;
      }
    }
  }

  /**
   * Publishes the colors from an AddressableLEDBuffer of length 20 to the SmartDashboard as a
   * single strip, flipping its order so LED 0 appears last on the dashboard.
   *
   * @param baseKey NetworkTable key prefix (e.g., "StripA")
   * @param buffer The AddressableLEDBuffer containing the full LED data (20 LEDs)
   */
  public void publishLEDsToDashboardFlipped(String baseKey, AddressableLEDBuffer buffer) {
    int length = buffer.getLength(); // Should be 20

    // Publish all LEDs in reverse order so that LED 0 is "at the bottom"
    for (int i = 0; i < length; i++) {
      // Flip index: so LED 0 → last, LED 1 → second last, etc.
      int reversedIndex = length - 1 - i;

      // Retrieve color in 8-bit format
      var color8Bit = buffer.getLED8Bit(reversedIndex);

      // Convert to hex string (#RRGGBB)
      String hex = String.format("#%02X%02X%02X", color8Bit.red, color8Bit.green, color8Bit.blue);

      // Publish to SmartDashboard: e.g., "StripA/led0", "StripA/led1", ...
      SmartDashboard.putString(baseKey + "/led" + i, hex);
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
    setActiveBuffer(wlLEDBuffer);
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
    setActiveBuffer(wlLEDBuffer);
  }

  public void blink() {
    currentColor_GOAL = LED_EFFECTS.BLINK;
  }

  public void doBlink() {
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

    setActiveBuffer(wlLEDBuffer);
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
      setActiveBuffer(wlRedLEDBuffer);
    } else {
      setActiveBuffer(wlBlackLEDBuffer);
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
      switch (skittleCount) {
        case 0:
          setActiveBuffer(wlRedLEDBuffer);
          break;
        case 1:
          setActiveBuffer(wlOrangeLEDBuffer);
          break;
        case 2:
          setActiveBuffer(wlYellowLEDBuffer);
          break;
        case 3:
          setActiveBuffer(wlGreenLEDBuffer);
          break;
        case 4:
          setActiveBuffer(wlBlueLEDBuffer);
          break;
        case 5:
          setActiveBuffer(wlIndigoLEDBuffer);
          break;
        case 6:
          setActiveBuffer(wlVioletLEDBuffer);
          break;
        default:
          break;
      }
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
      setActiveBuffer(wlWhiteLEDBuffer);
    } else {
      setActiveBuffer(wlPurpleLEDBuffer);
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
    setActiveBuffer(wlLEDBuffer);
  }

  private void setElevatorHeightEffect(CoralSystemPresets preset, Color brightColor) {
    final int totalLEDs = wlLEDBuffer.getLength(); // 20

    int brightStart;
    int brightCount;
    switch (preset) {
      case STOW -> {
        currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
        return;
      }
      case L1 -> {
        brightStart = 0;
        brightCount = 5;
      }
      case L2 -> {
        brightStart = 5;
        brightCount = 5;
      }
      case L3 -> {
        brightStart = 10;
        brightCount = 5;
      }
      case L4 -> {
        brightStart = 15;
        brightCount = 19;
      }
      default -> {
        brightStart = 0;
        brightCount = 0;
      }
    }

    // 1) Fill the entire 20-LED strip with white color
    for (int i = 0; i < totalLEDs; i++) {
      wlLEDBuffer.setLED(i, Color.kWhite);
    }

    // 2) Overwrite the bright block in the designated range
    for (int i = brightStart; i < brightStart + brightCount && i < totalLEDs; i++) {
      wlLEDBuffer.setLED(i, brightColor);
    }

    setActiveBuffer(wlLEDBuffer);
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
    setActiveBuffer(wlLEDBuffer);

    // Increment the counter and reset the timer.
    counter++;
    effectTimer.reset();
  }

  public void toggleBranchAlignmentIndicator() {
    branchAlignmentOn = !branchAlignmentOn;
  }

  private void setActiveBuffer(AddressableLEDBuffer buffer) {
    currentActiveBuffer = buffer;
    wlLED.setData(buffer);
  }

  public void updateBlinkPeriod(double lateralError) {
    // Clamp lateralError to be at least MIN_TOLERANCE_BLINK.
    if (lateralError <= MIN_TOLERANCE_BLINK) {
      currentBlinkPeriod = MIN_BLINK_PERIOD;
    } else if (lateralError >= MAX_TOLERANCE_BLINK) {
      currentBlinkPeriod = MAX_BLINK_PERIOD;
    } else {
      double fraction =
          (lateralError - MIN_TOLERANCE_BLINK) / (MAX_TOLERANCE_BLINK - MIN_TOLERANCE_BLINK);
      currentBlinkPeriod = MIN_BLINK_PERIOD + fraction * (MAX_BLINK_PERIOD - MIN_BLINK_PERIOD);
    }
    Logger.recordOutput("Alignment/Branch/DynamicBlink/currentBlinkPeriod", currentBlinkPeriod);
  }

  public void dynamicBlink() {
    double timeStamp = Timer.getFPGATimestamp();

    // Initialize timing if necessary.
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
      lastTime = timeStamp;
    }

    // Check if it's time to toggle based on currentBlinkPeriod.
    if (timeStamp - lastTime >= currentBlinkPeriod) {
      on = !on;
      lastTime = timeStamp;

      // Show green when "on" and black when "off"
      if (on) {
        setActiveBuffer(wlGreenLEDBuffer);
      } else {
        setActiveBuffer(wlBlackLEDBuffer);
      }
    }
  }

  private LED_EFFECTS updateLightingGoal() {

    if (DriverStation.isDisabled()) {
      return LED_EFFECTS.RSL; // implement RSL lighting
    }

    if (RobotStatus.isClimbingFinished()) return LED_EFFECTS.SEGMENTPARTY;

    if (RobotStatus.justPickedUpCoral() && !pickupBlinkTriggered) {
      pickupBlinkTriggered = true;
      return LED_EFFECTS.BLINK;
    }

    if (this.getCurrentCoralPreset.get() == CoralSystemPresets.PICKUP && !RobotStatus.haveCoral()) {
      return LED_EFFECTS.SEARCH_LIGHT;
    }

    if (RobotStatus.justScoredCoral()) {
      pickupBlinkTriggered = false;
      return LED_EFFECTS.PURPLE;
    }

    if (RobotStatus.justMissedCoralScoral()) {
      pickupBlinkTriggered = false;
      return LED_EFFECTS.BLUEOMBRE;
    }

    return LED_EFFECTS.BLUEOMBRE;
  }
}
