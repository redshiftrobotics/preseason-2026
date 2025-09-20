package frc.robot.subsystems.led;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.led.LEDConstants.BlinkenMode;

public class BlinkenLEDStripIO implements LEDStripIO {

  private final PWM pwm;
  private final BlinkenMode mode;

  private BlinkenLEDPattern pattern;

  private final Debouncer setupTimeDebouncer = new Debouncer(0.8);

  public BlinkenLEDStripIO(int pwmPort, BlinkenMode mode, BlinkenLEDPattern initialPattern) {
    pwm = new PWM(pwmPort);
    this.mode = mode;
    pattern = initialPattern;
  }

  @Override
  public void updateInputs(LEDStripIOInputs inputs) {
    inputs.hasSetup = setupTimeDebouncer.calculate(RobotState.isEnabled());

    if (inputs.hasSetup) {
      inputs.targetPulse = pattern.getPulse();
    } else {
      inputs.targetPulse = mode.setupPulse;
    }

    inputs.measuredPulse = pwm.getPulseTimeMicroseconds();

    inputs.requestedPattern = pattern;

    pwm.setPulseTimeMicroseconds(inputs.targetPulse);
  }

  @Override
  public void setPattern(BlinkenLEDPattern pattern) {
    this.pattern = pattern;
  }
}
