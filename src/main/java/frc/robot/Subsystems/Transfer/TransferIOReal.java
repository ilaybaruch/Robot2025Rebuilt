package frc.robot.Subsystems.Transfer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class TransferIOReal implements TransferIO {

    public POMSparkMax motor;
    public POMDigitalInput limitSwitch;

    public TransferIOReal() {
        motor = new POMSparkMax(0);
        limitSwitch = new POMDigitalInput(0);
    }

    @Override
    public void updateInputs(TransferInputs inputs) {

    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void stopMotor() {
        motor.stop();
    }

    @Override
    public boolean isCoralIn() {
        return limitSwitch.get();
    }

}
