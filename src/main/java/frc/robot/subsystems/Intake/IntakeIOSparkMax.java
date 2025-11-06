package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.GEAR_RATIO;
import static frc.robot.subsystems.intake.IntakeConstants.MOTOR1_ID;
import static frc.robot.subsystems.intake.IntakeConstants.MOTOR2_ID;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax Motor1 = new SparkMax(MOTOR1_ID, MotorType.kBrushless);
  private final SparkMax Motor2 = new SparkMax(MOTOR2_ID, MotorType.kBrushless);
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
          Motor1.getAppliedOutput() * Motor1.getBusVoltage(),
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
