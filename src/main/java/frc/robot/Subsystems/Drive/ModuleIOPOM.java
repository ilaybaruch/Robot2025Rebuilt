package frc.robot.Subsystems.Drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Subsystems.Drive.DriveConstants.*;
// import static frc.robot.Subsystems.Drive.OdometryThread.*;

public class ModuleIOPOM implements ModuleIO {

        private final int module;

        private final Timer resetToAbsoluteTimer = new Timer();

        // hardware
        private final TalonFX driveMotor;
        private final SparkMax turnMotor;
        private final CANcoder turnEncoder;

        // closed loop controller
        private final SparkClosedLoopController turnController;
        private final Slot0Configs driveMotorGains;

        private final DrivePIDTuning drivePIDTuning;

        private final SparkMaxConfig turnConfig;
        private final TalonFXConfiguration driveConfig;

        // // Queue inputs from odometry thread
        private final Queue<Double> timestampQueue;
        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;

        // Connection debouncers
        private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
        private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

        private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

        public ModuleIOPOM(int module) {
                this.module = module;

                Rotation2d zeroRotation = switch (module) {
                        case 0 -> frontLeftZeroRotation;
                        case 1 -> frontRightZeroRotation;
                        case 2 -> backLeftZeroRotation;
                        case 3 -> backRightZeroRotation;
                        default -> new Rotation2d();
                };

                // hardware
                driveMotor = new TalonFX(swerveBaseID + swerveModuleIDsCount * module);
                turnMotor = new SparkMax(swerveBaseID + 1 + swerveModuleIDsCount * module, MotorType.kBrushless);

                turnEncoder = new CANcoder(swerveBaseID + 2 + swerveModuleIDsCount * module);

                drivePIDTuning = new DrivePIDTuning();

                // turn encouder config
                var encoderConfig = new CANcoderConfiguration();
                encoderConfig.MagnetSensor.SensorDirection = module == 1
                                ? SensorDirectionValue.CounterClockwise_Positive
                                : SensorDirectionValue.Clockwise_Positive;
                encoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();
                turnEncoder.getConfigurator().apply(encoderConfig, 0.25);

                turnController = turnMotor.getClosedLoopController();

                // drive motor config
                driveConfig = new TalonFXConfiguration();
                driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                driveMotorGains = new Slot0Configs()
                                .withKP(driveKp).withKI(driveKi).withKS(driveKs).withKV(driveKv);
                // closed loop function
                driveConfig.Slot0 = driveMotorGains;
                driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
                driveConfig.Feedback.RotorToSensorRatio = 1.0;
                driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveSlipCurrent;
                driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveSlipCurrent;
                driveConfig.CurrentLimits.StatorCurrentLimit = driveSlipCurrent;
                driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = driveRampRate;
                driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = driveRampRate;
                driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = driveRampRate;

                driveMotor.getConfigurator().apply(driveConfig);

                // Configure turn motor
                turnConfig = new SparkMaxConfig();
                turnConfig
                                .inverted(turnInverted)
                                .idleMode(IdleMode.kCoast)
                                .smartCurrentLimit(turnMotorCurrentLimit)
                                .voltageCompensation(12.0)
                                .closedLoopRampRate(turnMotorRampRate)
                                .openLoopRampRate(turnMotorRampRate);
                turnConfig.encoder
                                // .inverted(turnEncoderInverted)
                                .positionConversionFactor(turnEncoderPositionFactor)
                                .velocityConversionFactor(turnEncoderVelocityFactor)
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);
                turnConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(turnKp, 0, turnKd, 0, ClosedLoopSlot.kSlot0)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                                .outputRange(-0.30, 0.30);
                turnConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);

                turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                // Create odometry queues
                timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
                drivePositionQueue = OdometryThread.getInstance()
                                .registerSignal(() -> Units
                                                .rotationsToRadians(driveMotor.getPosition().getValueAsDouble()));
                turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor,
                                turnMotor.getEncoder()::getPosition);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
                var driveStatus = BaseStatusSignal.refreshAll(
                                driveMotor.getPosition(),
                                driveMotor.getVelocity(),
                                driveMotor.getMotorVoltage(),
                                driveMotor.getStatorCurrent());
                inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
                inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
                inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
                inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
                inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

                inputs.absolutePosition = getAbsolutePosition();

                // Update odometry inputs
                inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
                inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value)
                                .toArray();
                inputs.odometryTurnPositions = turnPositionQueue.stream()
                                .map((Double value) -> new Rotation2d(value))
                                .toArray(Rotation2d[]::new);
                timestampQueue.clear();
                drivePositionQueue.clear();
                turnPositionQueue.clear();

                inputs.turnPosition = new Rotation2d(getAbsolutePosition());

                if (resetToAbsoluteTimer.get() > 2) {
                        resetToAbsoluteTimer.restart();
                        resetToAbsolute();
                }
                if (!resetToAbsoluteTimer.isRunning()) {
                        resetToAbsoluteTimer.start();
                }
        }

        public void resetToAbsolute() {
                SparkUtil.tryUntilOk(
                                turnMotor,
                                5,
                                () -> turnMotor
                                                .getEncoder()
                                                .setPosition(getAbsolutePosition()));
        }

        @Override
        public void setDriveOpenLoop(double output) {
                driveMotor.setVoltage(output);
        }

        @Override
        public void setTurnOpenLoop(double output) {
                turnMotor.setVoltage(output);
        }

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {
                double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
                driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
        }

        @Override
        public void setTurnPosition(Rotation2d setpoint) {
                double error = setpoint.minus(Rotation2d.fromRadians(getAbsolutePosition())).getRadians();
                if (Math.abs(error) >= Math.PI) {
                        error -= Math.copySign(Math.PI, error);
                        error *= -1;
                }
                var ks = Math.copySign(turnKs, error);
                if (Math.abs(error) > 0.03) {
                        turnController.setReference(setpoint.getRadians(), ControlType.kPosition,
                                        ClosedLoopSlot.kSlot0, ks, ArbFFUnits.kVoltage);
                } else {
                        setTurnOpenLoop(0);
                }
        }

        @Override
        public double getAbsolutePosition() {
                return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        }

        @Override
        public void setPIDValues() {
                turnConfig.closedLoop.pidf(drivePIDTuning.getTurnKp(), 0, drivePIDTuning.getTurnKd(),
                                drivePIDTuning.getTurnFF(),
                                ClosedLoopSlot.kSlot0);
                turnMotor.configure(turnConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                driveMotorGains.withKP(drivePIDTuning.getDriveKp()).withKD(drivePIDTuning.getDriveKi())
                                .withKS(drivePIDTuning.getDriveKs()).withKV(drivePIDTuning.getDriveKv());
                driveConfig.Slot0 = driveMotorGains;
        }
}
