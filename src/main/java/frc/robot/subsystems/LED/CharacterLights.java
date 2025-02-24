package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color;

public class CharacterLights {

  private static final Color RED = new Color(212, 3, 44);
  private static final Color DARK_RED = new Color(255, 77, 0);
  private static final Color WHITE = new Color(255, 251, 247);
  private static final Color SKIN = new Color(255, 195, 156);
  private static final Color SKIN2 = new Color(254, 213, 186);
  private static final Color BOWSER_SKIN = new Color(254, 195, 147);

  private static final Color BLACK = new Color(31, 24, 12);
  private static final Color YELLOW = new Color(252, 247, 0);
  private static final Color YELLOW2 = new Color(255, 231, 0);
  private static final Color LIGHT_BLUE = new Color(3, 192, 251);
  private static final Color BLUE = new Color(1, 113, 193);
  private static final Color BROWN = new Color(121, 57, 33);

  private static final Color YOSHI_GREEN = new Color(71, 202, 1);
  private static final Color GREEN = new Color(2, 159, 35);

  private static final Color LIGHT_PINK = new Color(254, 117, 202);
  private static final Color DARK_PINK = new Color(237, 91, 149);
  private static final Color PURPLE = new Color(158, 40, 148);
  private static final Color DARK_GREEN = new Color(0, 131, 66);

  private static final Color DARK_PURPLE = new Color(100, 48, 154);
  private static final Color DARK_BLUE = new Color(24, 39, 82);

  private static final Color DARK_ORANGE = new Color(255, 100, 0);
  private static final Color ORANGE = new Color(255, 105, 180);
  private static final Color LIGHT_ORANGE = new Color(253, 180, 0);
  private static final Color CHARCOAL = new Color(64, 64, 64);

  // Reversed Mario Colors
  public static final Color[] MARIO_COLORS = {
    BROWN, BLUE, YELLOW, BLUE, RED, SKIN, BLACK, SKIN, RED, WHITE, RED, BLACK
  };

  // Reversed LEDs Per Color
  public static final int[] MARIO_LEDS_PER_COLOR = {1, 5, 1, 1, 2, 2, 1, 2, 1, 1, 1, 2};

  // WARIO
  public static final Color[] WARIO_COLORS = {
    DARK_GREEN, PURPLE, YELLOW, PURPLE, YELLOW, SKIN, BLACK, DARK_PINK, SKIN, YELLOW, WHITE, YELLOW
  };
  public static final int[] WARIO_LEDS_PER_COLOR = {1, 5, 1, 1, 2, 3, 1, 1, 2, 1, 1, 1};

  // LUIGI
  public static final Color[] LUIGI_COLORS = {
    BROWN,
    BLUE,
    YELLOW,
    BLUE,
    GREEN,
    SKIN,
    BLACK,
    SKIN, // skin
    GREEN, // hat
    WHITE, // hat
    GREEN // hat
  };
  public static final int[] LUIGI_LEDS_PER_COLOR = {1, 7, 1, 1, 2, 2, 1, 2, 1, 1, 1};

  // Princess Peach
  public static final Color[] PEACH_COLORS = {
    DARK_PINK,
    LIGHT_PINK,
    DARK_PINK,
    LIGHT_PINK,
    YELLOW,
    LIGHT_BLUE,
    YELLOW,
    LIGHT_PINK,
    DARK_PINK,
    SKIN2,
    YELLOW2,
    ORANGE
  };
  public static final int[] PEACH_LEDS_PER_COLOR = {1, 6, 1, 1, 1, 1, 1, 1, 1, 1, 3, 2};

  // YOSHI
  public static final Color[] YOSHI_COLORS = {
    YELLOW,
    DARK_ORANGE,
    YOSHI_GREEN,
    WHITE,
    RED,
    YOSHI_GREEN,
    WHITE, // Yellow (hair)
    YOSHI_GREEN
  };
  public static final int[] YOSHI_LEDS_PER_COLOR = {1, 2, 2, 8, 1, 3, 2, 1};

  // TOAD
  public static final Color[] TOAD_COLORS = {
    BROWN,
    WHITE,
    DARK_BLUE,
    YELLOW,
    SKIN,
    YELLOW,
    DARK_BLUE,
    SKIN,
    WHITE,
    RED,
    WHITE,
    BLACK // TOAD IS SHORT
  };
  public static final int[] TOAD_LEDS_PER_COLOR = {1, 3, 1, 1, 1, 1, 1, 2, 1, 2, 1, 5};

  // BOWSER
  public static final Color[] BOWSER_COLORS = {
    WHITE, LIGHT_ORANGE, BOWSER_SKIN, LIGHT_ORANGE, CHARCOAL, BOWSER_SKIN, DARK_GREEN, RED
  };
  public static final int[] BOWSER_LEDS_PER_COLOR = {1, 1, 7, 1, 1, 3, 3, 1};

  // WALUIGI
  public static final Color[] WALUIGI_COLORS = {
    DARK_RED,
    DARK_BLUE,
    YELLOW,
    DARK_BLUE,
    DARK_PURPLE,
    SKIN,
    BLACK,
    DARK_PINK,
    SKIN, // skin
    DARK_PURPLE, // hat
    WHITE, // hat
    DARK_PURPLE // hat
  };
  public static final int[] WALUIGI_LEDS_PER_COLOR = {1, 6, 1, 1, 2, 2, 1, 1, 2, 1, 1, 1};

  // Consolidating all characters' color arrays
  public static final Color[][] ALL_CHARACTERS_COLORS = {
    MARIO_COLORS,
    LUIGI_COLORS,
    PEACH_COLORS,
    YOSHI_COLORS,
    TOAD_COLORS,
    BOWSER_COLORS,
    WARIO_COLORS,
    WALUIGI_COLORS
  };

  // Consolidating all characters' LED distribution arrays
  public static final int[][] ALL_CHARACTERS_LEDS_PER_COLOR = {
    MARIO_LEDS_PER_COLOR,
    LUIGI_LEDS_PER_COLOR,
    PEACH_LEDS_PER_COLOR,
    YOSHI_LEDS_PER_COLOR,
    TOAD_LEDS_PER_COLOR,
    BOWSER_LEDS_PER_COLOR,
    WARIO_LEDS_PER_COLOR,
    WALUIGI_LEDS_PER_COLOR
  };
}
