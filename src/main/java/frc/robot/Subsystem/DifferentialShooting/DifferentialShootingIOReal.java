package frc.robot.Subsystem.DifferentialShooting;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DifferentialShootingIOReal implements DifferentialShootingIO {

    SparkMax frontMotor, backMotor;
    AbsoluteEncoder frontEncoder, backEncoder;

    public DifferentialShootingIOReal() {
        frontMotor = new SparkMax(0, MotorType.kBrushless);
        backMotor = new SparkMax(0, MotorType.kBrushless);
        frontEncoder = frontMotor.getAbsoluteEncoder();
        backEncoder = frontMotor.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(DifferentialShootingIOInputs inputs) {
        inputs.frontMotorVelocity = frontEncoder.getVelocity();
        inputs.backMotorVelocity = backEncoder.getVelocity();
        inputs.frontMotorAppliedVolts = frontMotor.getAppliedOutput() * frontMotor.getBusVoltage();
        inputs.backMotorAppliedVolts = backMotor.getAppliedOutput() * backMotor.getBusVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        frontMotor.setVoltage(voltage);
        backMotor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        frontMotor.set(speed);
        backMotor.set(speed);
    }

    @Override
    public void stopMotor() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public void setDifferentialVoltage(double voltage, double diff) {
        backMotor.setVoltage(voltage);
        frontMotor.setVoltage(voltage * diff);
    }

}
