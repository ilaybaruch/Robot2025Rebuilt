package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

public class ElevatorIOReal implements ElevatorIO {
    private final POMSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkBaseConfig config;
    private final POMDigitalInput limitSwitch;
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;
    private final ElevatorTuning elevatorTuning;

    public ElevatorIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        config = new SparkMaxConfig();
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, IS_SWITCH_NORMALLY_OPEN);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        feedforward = new ElevatorFeedforward(Ks, Kg, Kv);
        elevatorTuning = new ElevatorTuning();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION)
                .inverted(INVERTED);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);

        pidController.setTolerance(TOLERANCE);

        setPIDValues();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.output = motor.getAppliedOutput();
        inputs.atGoal = pidController.atGoal();
        inputs.velocity = encoder.getVelocity();
        inputs.isSwitchPressed = limitSwitch.get();
        inputs.position = encoder.getPosition();
        setPIDValues();
        resetEncouderIfPressed();
    }

    @Override
    public void setVolatge(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public boolean isPressed() {
        return limitSwitch.get();
    }

    @Override
    public void resetEncouderIfPressed() {
        if (isPressed()) {
            encoder.setPosition(0);
        }
    }

    @Override
    public void stopElevator() {
        setVolatge(feedforward.calculate(0));
    }

    @Override
    public void resistGravity() {
        setVolatge(feedforward.calculate(0));
    }

    @Override
    public void setGoal(double goal) {
        setVolatge(pidController.calculate(getPos(), goal)
                + feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public void resetPID() {
        pidController.reset(getPos(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), feedforward.calculate(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), feedforward.calculate(1)));
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
    public void setPIDValues() {
        pidController.setP(elevatorTuning.getKp());
        pidController.setI(elevatorTuning.getKi());
        pidController.setD(elevatorTuning.getKd());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(elevatorTuning.getMaxVelocity(), elevatorTuning.getMaxAcceleration()));
        feedforward.setKg(elevatorTuning.getKg());
        feedforward.setKv(elevatorTuning.getKv());
        feedforward.setKa(elevatorTuning.getKg());
    }
}
