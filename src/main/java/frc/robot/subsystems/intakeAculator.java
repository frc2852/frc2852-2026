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
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

public class intakeAculator extends SubsystemBase {
  
  // motor
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double setPosition;

  public intakeAculator() {

    motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);
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
    config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

    // Encoder
    config.encoder.positionConversionFactor(1 / IntakeActuatorConstants.GEAR_RATIO);
    config.encoder.velocityConversionFactor(1 / IntakeActuatorConstants.GEAR_RATIO);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeActuatorConstants.P, IntakeActuatorConstants.I, IntakeActuatorConstants.D);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void setPosition(double position) {
    setPosition = position;
    pid.setSetpoint(position, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - setPosition) <= IntakeActuatorConstants.POSITION_TOLERANCE_DEGREES;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
