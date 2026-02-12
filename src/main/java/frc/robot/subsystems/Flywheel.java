// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {

  private final TalonFX flywheelLeaderMotor, flywheelFollowerMotor;

  private final VelocityTorqueCurrentFOC velocityRequest;
  private final NeutralOut neutralRequest;

  private final StatusSignal<AngularVelocity> motorVelocity;

  private double targetVelocityRPM = 0.0;

  public Flywheel() {

    CANBus canBus = new CANBus(CANIds.CANIVORE);
    flywheelLeaderMotor = new TalonFX(CANIds.FLYWHEEL_LEADER_MOTOR, canBus); // Leader
    flywheelFollowerMotor = new TalonFX(CANIds.FLYWHEEL_FOLLOWER_MOTOR, canBus); // Follower

    velocityRequest = new VelocityTorqueCurrentFOC(0);
    neutralRequest = new NeutralOut();

    configureMotor();

    flywheelFollowerMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned)); // TO DO

    motorVelocity = flywheelFollowerMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(Constants.SIGNAL_UPDATE_FREQUENCY_HZ, motorVelocity, flywheelLeaderMotor.getPosition(),
        flywheelLeaderMotor.getMotorVoltage());

    flywheelLeaderMotor.optimizeBusUtilization();
    flywheelFollowerMotor.optimizeBusUtilization();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Feedback sensor configuration
    config.Feedback.RotorToSensorRatio = FlywheelConstants.GEAR_RATIO;

    // PIDF gains
    config.Slot0.kS = FlywheelConstants.S;
    config.Slot0.kV = FlywheelConstants.V;
    config.Slot0.kA = FlywheelConstants.A;
    config.Slot0.kP = FlywheelConstants.P;
    config.Slot0.kI = FlywheelConstants.I;
    config.Slot0.kD = FlywheelConstants.D;

    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.SUPPLY_CURRENT_LOWER_TIME;
    config.TorqueCurrent.PeakForwardTorqueCurrent = FlywheelConstants.PEAK_FORWARD_TORQUE_CURRENT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = FlywheelConstants.PEAK_REVERSE_TORQUE_CURRENT;

    // Apply with retry
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = flywheelLeaderMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.err.println("Failed to configure Flywheel motor: " + status);
    }
  }

  // Set target velocity in RPM
  public void setVelocity(double rpm) {
    targetVelocityRPM = clamp(rpm, FlywheelConstants.MIN_RPM, FlywheelConstants.MAX_RPM);
    double rps = targetVelocityRPM / 60.0;
    flywheelLeaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  // Flywheel at target Velocity?
  public boolean atVelocity() {
    double currentRPM = getCurrentVelocityRPM();
    return Math.abs(currentRPM - targetVelocityRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  // Get current velocity in RPM
  public double getCurrentVelocityRPM() {
    BaseStatusSignal.refreshAll(motorVelocity);
    return motorVelocity.getValue().in(RotationsPerSecond) * 60.0;
  }

  // Clamp utility
  public double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
