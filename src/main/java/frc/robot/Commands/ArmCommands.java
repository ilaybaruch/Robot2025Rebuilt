package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Arm.Arm;
import static frc.robot.Subsystems.Arm.ArmConstants.*;

public class ArmCommands extends Command {

    public Command setVoltage(double voltage, Arm arm) {
        return Commands.runEnd(
                () -> arm.getIO().setVoltage(voltage),
                () -> arm.getIO().resistGravity(), arm);
    }

    public Command goToPositon(double goal, Arm arm) {
        return new FunctionalCommand(() -> {
            arm.getIO().resistGravity();
            arm.getIO().resetPID(goal);
        },
                () -> arm.getIO().setGoal(goal),
                interrupted -> arm.getIO().resistGravity(),
                () -> arm.getIO().atGoal(), arm);
    }

    public Command closeArm(Arm arm) {
        return Commands.sequence(
                goToPositon(ARM_CLOSE_POS, arm),
                setVoltage(-1, arm).until(arm.getIO()::downSwitchPressed));
    }

}
