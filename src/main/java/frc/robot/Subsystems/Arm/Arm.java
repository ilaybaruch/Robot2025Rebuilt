package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ArmCommands;

public class Arm extends SubsystemBase {

    ArmIO armIO;
    ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
        setDefaultCommand(new RepeatCommand(this.runOnce(
                () -> armIO.resistGravity())));
    }

    public ArmIO getIO() {
        return armIO;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);// weird thing with IOInput and inputs need to check
        Logger.processInputs("Arm", inputs);
    }

}
