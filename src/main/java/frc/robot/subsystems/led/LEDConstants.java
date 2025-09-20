package frc.robot.subsystems.led;

public class LEDConstants {
  public enum BlinkenMode {
    STRIP_5V(2125),
    STRIP_12V(2145);

    public final int setupPulse;

    BlinkenMode(int zeroPulseWidth) {
      this.setupPulse = zeroPulseWidth;
    }
  }

  public record LEDConfig(int port, BlinkenMode mode) {}

  public static final LEDConfig PWM_0_5V_STRIP = new LEDConfig(0, BlinkenMode.STRIP_5V);
  public static final LEDConfig PWM_1_5V_STRIP = new LEDConfig(1, BlinkenMode.STRIP_5V);

  public static final LEDConfig PWM_0_12V_STRIP = new LEDConfig(0, BlinkenMode.STRIP_12V);
  public static final LEDConfig PWM_1_12V_STRIP = new LEDConfig(1, BlinkenMode.STRIP_12V);
}
