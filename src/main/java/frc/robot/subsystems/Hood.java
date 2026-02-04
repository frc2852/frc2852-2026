// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

     //motor
    private final TalonFX motor;
    private final CANcoder encoder;

    //control
    private final MotionMagicTorqueCurrentFOC positionRequest;
    private final NeutralOut nuetralRequest;
    private final VoltageOut voltageoutRequest;

    //Status Signals
    private final StatusSignal<Angle> motorposition;
    private final StatusSignal<Angle> cancoderposition;

    //State
    private double targetPosition = 0.0;

  public Hood() {

      //Motor and Encoder Initialization
      CANBus canbus = new CANBus(CANIds.CANIVORE);
      motor = new TalonFX(CANIds.HOOD_MOTOR, canbus);
      encoder = new CANcoder(CANIds.HOOD_CANCODER, canbus);

      //Control
      positionRequest = new MotionMagicTorqueCurrentFOC(0);
      nuetralRequest = new NeutralOut ();
      voltageoutRequest = new VoltageOut(0);

      //Configuration
      configureCANcoder();
      configureMotor();

      //Status Signals
      motorposition = motor.getPosition();
      cancoderposition = encoder.getPosition();

      BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        motorposition
      , cancoderposition
      , motor.getVelocity(),
      motor.getMotorVoltage());

      //canbus opptimization
      motor.optimizeBusUtilization();
      encoder.optimizeBusUtilization();
    }

    private void configureCANcoder() {
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      config.MagnetSensor.MagnetOffset = Constants.HoodConstants.CANCODER_OFFSET; //Const
      encoder.getConfigurator().apply(config);
    }

    private void configureMotor() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      //Motor Output
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      //feedback
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      config.Feedback.FeedbackRemoteSensorID = CANIds.HOOD_CANCODER;
      config.Feedback.RotorToSensorRatio = HoodConstants.GEAR_RATIO;
      config.Feedback.SensorToMechanismRatio = 1.0;

      //PID gains
      config.Slot0.kS = HoodConstants.S;
      config.Slot0.kV = HoodConstants.V;
      config.Slot0.kA = HoodConstants.A;
      config.Slot0.kP = HoodConstants.P;
      config.Slot0.kI = HoodConstants.I;
      config.Slot0.kD = HoodConstants.D;
      config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      config.Slot0.kG = 0.0; 

      //Motion Magic
      // Convert deg-based constants to rotations-based units for Motion Magic
      config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY / 360.0;
      config.MotionMagic.MotionMagicAcceleration = HoodConstants.MOTION_MAGIC_ACCELERATION / 360.0;
      config.MotionMagic.MotionMagicJerk = HoodConstants.MOTION_MAGIC_JERK / 360.0;

      //Software Limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_POSITION_DEGREES / 360.0;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_POSITION_DEGREES / 360.0;

      //Current Limits
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.SUPPLY_CURRENT_LOWER_LIMIT;
      config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.SUPPLY_CURRENT_LOWER_TIME; 

      //Apply w/ retry
      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; i++) {
        status = motor.getConfigurator().apply(config);
        if (status.isOK()) {
          break;
        }
      }
      if (!status.isOK()) {
        System.out.println("Failed to configure Hood motor: " + status);
      }
    }

    public void setPosition(double degree) {
      targetPosition = clamp(degree, HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES); 
      double rotations =  targetPosition / 360.0;
      motor.setControl(positionRequest.withPosition(rotations));
    }

    public boolean isAtPosition() {
      double currentDegrees = getCurrentPositionDegrees();
      return Math.abs(currentDegrees - targetPosition) < Constants.HoodConstants.POSITION_TOLERANCE_DEGREES;
    }
    
    public double getCurrentPositionDegrees() {
      BaseStatusSignal.refreshAll(motorposition);
      return motorposition.getValue().in(Rotation) * 360.0;
    }

    public double clamp(double value, double min, double max) {
     return Math.max(min, Math.min(max, value));
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
