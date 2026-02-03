// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeAculator extends SubsystemBase {
  
  // motor
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double setPosition;
  private double tolerance = 10;

 //P, I, D constants
  private static final double P = 0.1;
  private static final double I = 0.0;
  private static final double D = 0.0;

  public intakeAculator() {

    motor = new SparkFlex(13, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();
    configureMotor();
  }

  public void configureMotor() {
    SparkFlexConfig config = new SparkFlexConfig();

    // Motor output
    config.idleMode(IdleMode.kCoast);
    config.inverted(false);

    // Current Limits
    config.smartCurrentLimit(60);
    config.secondaryCurrentLimit(70);

    // Encoder
    double INTAKE_GEAR_RATIO = 1;
    config.encoder.positionConversionFactor(1 / INTAKE_GEAR_RATIO);
    config.encoder.velocityConversionFactor(1 / INTAKE_GEAR_RATIO);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, I, D); // Const P, I, D
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void setPosition(double position) {
    setPosition = position;
    pid.setSetpoint(position, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - setPosition) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
