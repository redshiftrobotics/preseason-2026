package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.GEAR_RATIO;
import static frc.robot.subsystems.intake.IntakeConstants.MOTOR1_ID;
import static frc.robot.subsystems.intake.IntakeConstants.MOTOR2_ID;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax motor1 = new SparkMax(MOTOR1_ID, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(MOTOR2_ID, MotorType.kBrushless);
  private final SparkBaseConfig config;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  public IntakeIOSparkMax() {
    leftEncoder = motor1.getEncoder();
    rightEncoder = motor2.getEncoder();

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts =
        new double[] {
          motor1.getAppliedOutput() * motor1.getBusVoltage(),
        };
    inputs.leftSupplyCurrentAmps = new double[] {motor1.getOutputCurrent()};

    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightAppliedVolts =
        new double[] {
          motor2.getAppliedOutput() * motor2.getBusVoltage(),
        };
    inputs.rightSupplyCurrentAmps = new double[] {motor1.getOutputCurrent()};
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    motor1
        .getClosedLoopController()
        .setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
    motor2
        .getClosedLoopController()
        .setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    motor1.set(0);
    motor2.set(0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}