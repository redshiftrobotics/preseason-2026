package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDStripIO.LEDStripIOInputs;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  private LEDStripIO[] strips;
  private LEDStripIOInputsAutoLogged[] inputs;

  public LEDSubsystem(LEDStripIO... strips) {
    this.strips = strips;

    inputs =
        Arrays.stream(strips)
            .map(s -> new LEDStripIOInputs())
            .toArray(LEDStripIOInputsAutoLogged[]::new);

    Logger.recordOutput("led/numStrips", strips.length);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < strips.length; i++) {
      strips[i].updateInputs(inputs[i]);
      Logger.processInputs("LED/strip" + i, inputs[i]);
    }
  }

  public Command runColor(BlinkenLEDPattern color) {
    return run(() -> set(color));
  }

  public Command runColor(Supplier<BlinkenLEDPattern> color) {
    return run(() -> set(color.get()));
  }

  public Command runColor(
      BlinkenLEDPattern colorIfBlue,
      BlinkenLEDPattern colorIfRed,
      BlinkenLEDPattern colorIfUnknown) {
    return runColor(
        () ->
            DriverStation.getAlliance()
                .map(a -> a == Alliance.Blue ? colorIfBlue : colorIfRed)
                .orElse(colorIfUnknown));
  }

  public Command runNoColor() {
    return runColor(BlinkenLEDPattern.OFF);
  }

  public void set(BlinkenLEDPattern pattern) {
    for (int i = 0; i < strips.length; i++) {
      set(i, pattern);
    }
  }

  public void set(int stripIndex, BlinkenLEDPattern pattern) {
    strips[stripIndex].setPattern(pattern);
  }
}
