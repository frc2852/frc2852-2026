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
      CANBus canbus = new CANBus("canbus");
      motor = new TalonFX(24, canbus);
      encoder = new CANcoder(25, canbus);

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
      config.MagnetSensor.MagnetOffset = 0.0;
      encoder.getConfigurator().apply(config);
    }

    private void configureMotor() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      //Motor Output
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      //feedback
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      config.Feedback.FeedbackRemoteSensorID = 25; //Const
      config.Feedback.RotorToSensorRatio = 1.0; //Const
      config.Feedback.SensorToMechanismRatio = 1.0;

      //PID gains
      config.Slot0.kS = 0.0; //Const
      config.Slot0.kV = 0.0; //Const
      config.Slot0.kA = 2.0; //Const
      config.Slot0.kP = 0.0; //Const
      config.Slot0.kI = 0.0; //Const
      config.Slot0.kD = 0.0; //Const
      config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      config.Slot0.kG = 0.0; 

      //Motion Magic
      config.MotionMagic.MotionMagicCruiseVelocity = 1 /360.0; //Const
      config.MotionMagic.MotionMagicAcceleration = 1 / 360.0; //Const
      config.MotionMagic.MotionMagicJerk = 1 / 360.0; //Const

      //Software Limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1 / 360.0; //Const
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1 / 360.0; //Const

      //Current Limits
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 1;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 1; //Const
      config.CurrentLimits.SupplyCurrentLowerLimit = 1; //Const
      config.CurrentLimits.SupplyCurrentLowerTime = 1; //Const

      // Torque current limits
      config.TorqueCurrent.PeakForwardTorqueCurrent = 1; //Const
      config.TorqueCurrent.PeakReverseTorqueCurrent = 1; //Const

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
      targetPosition = clamp(degree, 0.0, 180.0); //Const min and max
      double rotations =  targetPosition / 360.0;
      motor.setControl(positionRequest.withPosition(rotations));
    }

    public boolean isAtPosition() {
      double currentDegrees = getCurrentPositionDegrees();
      return Math.abs(currentDegrees - targetPosition) < 1; //Const tolerance
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
