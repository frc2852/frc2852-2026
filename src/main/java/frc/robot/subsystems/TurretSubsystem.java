package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.CANIds;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor;
    private final CANcoder turretCancoder;

    private final MotionMagicTorqueCurrentFOC positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    private final StatusSignal<Angle> motorPosition;

    private double targetDegrees = 0.0;

    public TurretSubsystem() {

         CANBus canBus = new CANBus(CANIds.CANIVORE);

        turretMotor = new TalonFX(CANIds.TURRET_MOTOR, canBus);
        turretCancoder = new CANcoder(CANIds.TURRET_CANCODER, canBus);


        positionRequest = new MotionMagicTorqueCurrentFOC(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        configureCANCoder();
        configureMotor();

        motorPosition = turretMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            motorPosition,
            turretMotor.getVelocity(),
            turretMotor.getMotorVoltage()
        );

        turretMotor.optimizeBusUtilization();
        turretCancoder.optimizeBusUtilization();
    }

    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET;

        turretCancoder.getConfigurator().apply(config);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Fused CANcoder
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = turretCancoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = TurretConstants.GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0;

        // PIDF
        config.Slot0.kS = TurretConstants.S;
        config.Slot0.kV = TurretConstants.V;
        config.Slot0.kA = TurretConstants.A;
        config.Slot0.kG = TurretConstants.G;
        config.Slot0.kP = TurretConstants.P;
        config.Slot0.kI = TurretConstants.I;
        config.Slot0.kD = TurretConstants.D;

        // Motion Magic
        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = TurretConstants.MOTION_MAGIC_JERK;

        // Soft limits (convert degrees → rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            TurretConstants.MAX_POSITION_DEGREES / 360.0;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            TurretConstants.MIN_POSITION_DEGREES / 360.0;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.SUPPLY_CURRENT_LOWER_TIME;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = turretMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.err.println("Failed to configure turret motor: " + status);
        }
    }

    public void setPosition(double degrees) {
        targetDegrees = clamp(degrees,TurretConstants.MIN_POSITION_DEGREES,TurretConstants.MAX_POSITION_DEGREES);

        double rotations = targetDegrees / 360.0;

        turretMotor.setControl(positionRequest.withPosition(rotations));
    }

    public boolean atPosition() {
        return Math.abs(getCurrentPositionDegrees() - targetDegrees)< TurretConstants.POSITION_TOLERANCE_DEGREES;}

    public double getCurrentPositionDegrees() {
        BaseStatusSignal.refreshAll(motorPosition);
        return motorPosition.getValue().in(Rotation) * 360.0;
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
