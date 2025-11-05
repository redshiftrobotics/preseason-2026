package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  IntakeIO intakeIO;
  IntakeIOInputs inputs;

  public Intake(IntakeIO io) {
    this.intakeIO = io;
    this.inputs = new IntakeIOInputs();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
  }

  public void set(double speed) {
    intakeIO.set(speed);
  }

  public void stop() {    
    intakeIO.stop();
  }
}