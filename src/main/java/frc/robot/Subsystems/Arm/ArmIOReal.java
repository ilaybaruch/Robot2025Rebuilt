package frc.robot.Subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.Subsystems.Arm.ArmConstants.*;

public class ArmIOReal implements ArmIO {

    POMSparkMax motor;
    POMDigitalInput upLimitSwitch;
    POMDigitalInput downLimitSwitch;
    RelativeEncoder encoder;
    SparkBaseConfig config;
    ProfiledPIDController pidController;
    ArmFeedforward feedforward;
    ArmTuning tuning;

    public ArmIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        config = new SparkMaxConfig();
        upLimitSwitch = new POMDigitalInput(UP_LIMIT_SWITCH_CHANNEL, UP_SWITCH_NORMALLY_OPEN);
        downLimitSwitch = new POMDigitalInput(DOWN_LIMIT_SWITCH_CHANNEL, DOWN_SWITCH_NORMALLY_OPEN);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        tuning = new ArmTuning();

        config.idleMode(IdleMode.kBrake).inverted(INVERTED).smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);

        pidController.setTolerance(TOLERANCE);

    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getAppliedOutput();
        inputs.output = motor.getAppliedOutput();
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.upLimitSwitchPressed = upSwitchPressed();
        inputs.downLimitSwitchPressed = downSwitchPressed();
        inputs.atGoal = atGoal();
        setPIDValues();
        resetIfPressed();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public boolean upSwitchPressed() {
        return upLimitSwitch.get();
    }

    @Override
    public boolean downSwitchPressed() {
        return downLimitSwitch.get();
    }

    @Override
    public void resetIfPressed() {
        if (upSwitchPressed()) {
            encoder.setPosition(ARM_OPEN_POS);
        }

        if (downSwitchPressed()) {
            encoder.setPosition(ARM_CLOSE_POS);
        }

    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void resetPID() {
        pidController.reset(getPos(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(),
                    Math.max(encoder.getVelocity(), feedforward.calculate(getPos(), 1)));
        } else {
            pidController.reset(encoder.getPosition(),
                    Math.min(encoder.getVelocity(), feedforward.calculate(getPos(), 1)));
        }
    }

    @Override
    public void resistGravity() {
        motor.setVoltage(feedforward.calculate(getPos(), 0));
    }

    @Override
    public void setGoal(double goal) {
        motor.setVoltage(pidController.calculate(getPos(), goal) +
                feedforward.calculate(getPos(), pidController.getSetpoint().velocity));
    }

    @Override
    public void setPIDValues() {
        // pidController.setP(tuning.getKp());
        // pidController.setI(tuning.getKi());
        // pidController.setD(tuning.getKd());
        // pidController.setConstraints(
        // new TrapezoidProfile.Constraints(tuning.getMaxVelocity(),
        // tuning.getMaxAcceleration()));
        // feedforward = new ArmFeedforward(tuning.getKs(), tuning.getKg(),
        // tuning.getKv(),
        // tuning.getKa());
    }

    @Override
    public void addKg(double KG) {
        feedforward.setKg(KG + Kg);
    }

    @Override
    public boolean isHigh() {
        return encoder.getPosition() > 0;
    }

}
