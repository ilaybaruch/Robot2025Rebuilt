package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Arm.Arm;

public class ArmCommands extends Command {

    public Command RunArm(double speed, Arm arm) {
        return Commands.runEnd(() -> arm.getIO().setSpeed(speed), () -> arm.getIO().brakeElevator(), arm);
    }

    public Command ArmGoToPoistion(double goal, Arm arm) {
        return new FunctionalCommand(() -> {
            arm.getIO().stopElevator();
            arm.getIO().resetPID(goal);
        },
                () -> arm.getIO().goToPostion(goal), interrupted -> arm.getIO().stopElevator(),
                () -> arm.getIO().atGoal(), arm);
    }

    public Command ArmGoToStart(Arm arm) {
        return new FunctionalCommand(() -> {
            arm.getIO().stopElevator();
            arm.getIO().resetPID(0);
        },
                () -> arm.getIO().goToPostion(0),
                interrupted -> arm.getIO().stopElevator(),
                () -> arm.getIO().atGoal(),
                arm).andThen(() -> arm.getIO().setSpeed(-0.05)).until(() -> arm.getIO().getPos() == 0);
    }

}
