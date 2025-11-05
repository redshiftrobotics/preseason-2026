package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.examples.flywheel.FlywheelConstants.GEAR_RATIO;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;
import com.revrobotics.RelativeEncoder;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax Motor1 = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax Motor2 = new SparkMax(0, MotorType.kBrushless);
  private final SparkBaseConfig config;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  
  public IntakeIOSparkMax() {
    leftEncoder = Motor1.getEncoder();
    rightEncoder = Motor2.getEncoder();

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    Motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    Motor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts =
        new double[] {
          Motor1.getAppliedOutput() *Motor1.getBusVoltage(),
        };
    inputs.leftSupplyCurrentAmps = new double[] {Motor1.getOutputCurrent()};

    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightAppliedVolts =
        new double[] {
          Motor2.getAppliedOutput() * Motor2.getBusVoltage(),
        };
    inputs.rightSupplyCurrentAmps = new double[] {Motor1.getOutputCurrent()};
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