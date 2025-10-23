package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorIOReal implements ElevatorIO {
    private final POMSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkBaseConfig config;
    private final POMDigitalInput limitSwitch;
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;

    public ElevatorIOReal() {
        motor = new POMSparkMax(0);
        encoder = motor.getEncoder();
        config = new SparkMaxConfig();
        limitSwitch = new POMDigitalInput(0, false);
        pidController = new ProfiledPIDController(0, 0, 0, null);
        feedforward = new ElevatorFeedforward(0, 0, 0);

        // need to config
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.output = motor.getAppliedOutput();
        inputs.atGoal = pidController.atGoal();
        inputs.isSwitchPressed = limitSwitch.get();
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
    public void setFF() {
        setVolatge(feedforward.calculate(0));
    }

    @Override
    public void setPIDWithFF(double goal) {
        setVolatge(pidController.calculate(getPos(), goal)
                + feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public void resetPID() {
        pidController.reset(getPos());
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }
}
