// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.Commands.MultiSystemCommands;
import frc.robot.Commands.TransferCommands;
import frc.robot.Commands.ArmCommands;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOReal;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Transfer.TransferIOReal;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmIOReal;
import static frc.robot.Subsystems.Arm.ArmConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

        Elevator elevator;
        ElevatorCommands elevatorCommands;

        Arm arm;
        ArmCommands armCommands;

        Transfer transfer;
        TransferCommands transferCommands;

        MultiSystemCommands multiSystemCommands;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                elevator = new Elevator(new ElevatorIOReal());
                                elevatorCommands = new ElevatorCommands();
                                arm = new Arm(new ArmIOReal());
                                armCommands = new ArmCommands();
                                transfer = new Transfer(new TransferIOReal());
                                transferCommands = new TransferCommands();
                                multiSystemCommands = new MultiSystemCommands();
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                break;

                        default:
                                // Replayed robot, disable IO implementations

                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                driverController.rightTrigger().whileTrue(elevatorCommands
                                .setVoltage(() -> driverController.getRightTriggerAxis() * 2.5, elevator));
                driverController.leftTrigger().whileTrue(elevatorCommands
                                .setVoltage(() -> driverController.getLeftTriggerAxis() * -2, elevator));
                // driverController.a().onTrue(elevatorCommands.goToPosition(10, elevator));
                // driverController.x().onTrue(elevatorCommands.goToPosition(25, elevator));
                // driverController.b().onTrue(elevatorCommands.ElevatorDown(elevator));
                driverController.a().onTrue(armCommands.goToPositon(0, arm));
                driverController.x().onTrue(armCommands.goToPositon(ARM_OPEN_POS, arm));
                driverController.b().onTrue(armCommands.ArmGoToStart(arm));
                driverController.RB().whileTrue(armCommands.setVoltage(1, arm));
                driverController.LB().whileTrue(armCommands.setVoltage(-0.5, arm));
                driverController.PovLeft().onTrue(multiSystemCommands.IntakeSequence(transfer, elevator, arm));
                driverController.PovRight().onTrue(multiSystemCommands.L3(elevator, arm,
                                () -> driverController.PovRight().getAsBoolean()));
                driverController.PovDown().onTrue(multiSystemCommands.CloseAll(elevator, arm));
                driverController.PovUp().onTrue(
                                multiSystemCommands.L4(elevator, arm, () -> driverController.PovUp().getAsBoolean()));

        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
