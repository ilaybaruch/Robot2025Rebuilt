package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Arm.Arm;
import static frc.robot.Subsystems.Arm.ArmConstants.*;

public class ArmCommands extends Command {

    public Command RunArm(double voltage, Arm arm) {
        return Commands.runEnd(() -> arm.getIO().setVoltage(voltage), () -> arm.getIO().resistGravity(), arm);
    }

    public Command ArmGoToPoistion(double goal, Arm arm) {
        return new FunctionalCommand(() -> {
            arm.getIO().resistGravity();
            arm.getIO().resetPID(goal);
        },
                () -> arm.getIO().setGoal(goal), interrupted -> arm.getIO().resistGravity(),
                () -> arm.getIO().atGoal(), arm);
    }

    public Command ArmGoToStart(Arm arm) {
        return new FunctionalCommand(() -> {
            arm.getIO().resistGravity();
            arm.getIO().resetPID(ARM_CLOSE_POS);
        },
                () -> {arm.getIO().addKg(0);
                    arm.getIO().setGoal(ARM_CLOSE_POS);},
                interrupted -> arm.getIO().resistGravity(),
                () ->
                    arm.getIO().atGoal(),
                arm).andThen(() -> {arm.getIO().addKg(0);
                    arm.getIO().setVoltage(-0.5);}).until(() -> arm.getIO().getPos() == 0);
    }

}
