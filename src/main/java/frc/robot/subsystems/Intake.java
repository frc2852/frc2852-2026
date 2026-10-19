package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.frc2852.mechid.CharacterizableSubsystem;
import com.frc2852.mechid.MechIdConfig;
import com.frc2852.mechid.MotorModel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

public class Intake extends CharacterizableSubsystem {

    // Hardware
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkFlex vectorWheelRollerMotor;
    private final SparkFlex verticalRollerMotor;

    // Control requests
    private final DutyCycleOut dutyRequest;
    private final NeutralOut neutralRequest;

    // Status signals
    private final StatusSignal<AngularVelocity> leftVelocity;

    public Intake() {
        leftMotor = new TalonFX(CANIds.INTAKE_LEFT_MOTOR);
        rightMotor = new TalonFX(CANIds.INTAKE_RIGHT_MOTOR);
        vectorWheelRollerMotor = new SparkFlex(CANIds.VECTOR_WHEEL_ROLLER_MOTOR, MotorType.kBrushless);
        verticalRollerMotor = new SparkFlex(CANIds.VERTICAL_ROLLER_MOTOR, MotorType.kBrushless);

        dutyRequest = new DutyCycleOut(0);
        neutralRequest = new NeutralOut();

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive, "left");
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive, "right");
        configureRollerMotor(vectorWheelRollerMotor, IntakeConstants.VECTOR_WHEEL_ROLLER_INVERTED, "vector wheel");
        configureRollerMotor(verticalRollerMotor, IntakeConstants.VERTICAL_ROLLER_INVERTED, "vertical");

        leftVelocity = leftMotor.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                leftVelocity);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    private void configureMotor(TalonFX motor, InvertedValue inversion, String name) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inversion;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SUPPLY_CURRENT_LOWER_TIME;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure intake " + name + " motor: " + status);
        }
    }

    private void configureRollerMotor(SparkFlex motor, boolean inverted, String name) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(inverted);
        config.smartCurrentLimit(IntakeConstants.ROLLER_SMART_CURRENT_LIMIT);
        config.secondaryCurrentLimit(IntakeConstants.ROLLER_SECONDARY_CURRENT_LIMIT);

        REVLibError error = REVLibError.kError;
        for (int i = 0; i < 5; i++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
                break;
            }
        }
        if (error != REVLibError.kOk) {
            System.err.println("Failed to configure intake " + name + " roller motor: " + error);
        }
    }

    /**
     * MechID characterization setup. The two roller motors are independent (not
     * follower-configured), so both are driven with the identical voltage during the
     * routine — each motor's inversion config already makes +V = intake direction.
     * Run with the intake clear of game pieces.
     */
    @Override
    protected MechIdConfig configureMechId() {
        return MechIdConfig.flywheel("intake", leftMotor, MotorModel.KRAKEN_X60)
                .withSynchronizedMotors(rightMotor)
                .withMaxVelocityAbort(115.0 / IntakeConstants.GEAR_RATIO);
    }

    public void runIntake() {
        rightMotor.setControl(dutyRequest.withOutput(1));
        leftMotor.setControl(dutyRequest.withOutput(1));
        vectorWheelRollerMotor.set(IntakeConstants.VECTOR_WHEEL_ROLLER_SPEED);
        verticalRollerMotor.set(IntakeConstants.VERTICAL_ROLLER_SPEED);
    }

    public void runOuttake() {
        rightMotor.setControl(dutyRequest.withOutput(-1));
        leftMotor.setControl(dutyRequest.withOutput(-1));
        vectorWheelRollerMotor.set(-IntakeConstants.VECTOR_WHEEL_ROLLER_SPEED);
        verticalRollerMotor.set(-IntakeConstants.VERTICAL_ROLLER_SPEED);
    }

    public void stop() {
        rightMotor.setControl(neutralRequest);
        leftMotor.setControl(neutralRequest);
        vectorWheelRollerMotor.stopMotor();
        verticalRollerMotor.stopMotor();
    }

    public double getVelocityRPS() {
        BaseStatusSignal.refreshAll(leftVelocity);
        return leftVelocity.getValue().in(RotationsPerSecond);
    }

    public Command runCommand() {
        return run(this::runIntake)
                .finallyDo(this::stop)
                .withName("Intake.run");
    }

    public Command runOutCommand() {
        return run(this::runOuttake)
                .finallyDo(this::stop)
                .withName("IntakeOut.run");
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake/Velocity RPS", getVelocityRPS());
    }
}
