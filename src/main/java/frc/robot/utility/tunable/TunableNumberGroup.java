package frc.robot.utility.tunable;

public class TunableNumberGroup {
  private final String key;

  /**
   * Create a new LoggedTunableNumberGroup
   *
   * @param key Key on dashboard
   */
  public TunableNumberGroup(String key) {
    this.key = key;
  }

  // --- Create Number ---

  /**
   * Add a number to the group
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber number(String dashboardKey, double defaultValue) {
    return new TunableNumber(key + "/" + dashboardKey, defaultValue);
  }

  // --- Create Subgroup ---

  /**
   * Add a subgroup to the group
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumberGroup subgroup(String dashboardKey) {
    return new TunableNumberGroup(key + "/" + dashboardKey);
  }
}
