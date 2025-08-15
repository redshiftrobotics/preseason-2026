package frc.robot.utility.records;

import com.ctre.phoenix6.configs.Slot0Configs;

public record FeedForwardConstants(double kS, double kV, double kA, double kG) {
  public FeedForwardConstants(double kS, double kV, double kA) {
    this(kS, kV, kA, 0.0);
  }

  public FeedForwardConstants(Slot0Configs slot0Configs) {
    this(slot0Configs.kS, slot0Configs.kV, slot0Configs.kA, slot0Configs.kG);
  }
}
