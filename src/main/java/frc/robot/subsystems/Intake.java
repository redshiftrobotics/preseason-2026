package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax Motor1 = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax Motor2 = new SparkMax(0, MotorType.kBrushless);
  private final SparkBaseConfig config;

  public Intake() {
    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    Motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    Motor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void set(double speed) {
    Motor1.set(speed);
    Motor2.set(speed);
  }

  public void stop() {    
    Motor1.set(0);
    Motor2.set(0);
  }
}