package frc.robot.Subsystems.Transfer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.Subsystems.Transfer.TransferConstants.*;

public class TransferIOReal implements TransferIO {

    public POMSparkMax motor;
    public POMDigitalInput IRSensor;

    public TransferIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        IRSensor = new POMDigitalInput(IR_SENSOR_CHANNEL);
    }

    @Override
    public void updateInputs(TransferInputs inputs) {
        inputs.isCoralIn = isCoralIn();
        inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.output = motor.getAppliedOutput(); // hatol
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
    public void stopMotor() { // whaaaaaaa
        motor.stop();
    }

    @Override
    public boolean isCoralIn() {
        return IRSensor.get();
    }

}
