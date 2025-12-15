// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class DriveCommands {
        {
        }

        private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
                // Apply deadband
                double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
                Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

                // Square magnitude for more precise control
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Return new linear velocity
                return new Pose2d(new Translation2d(), linearDirection)
                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                .getTranslation();
        }

        public Command driveTune(Drive drive) {
                return Commands.runOnce(() -> drive.setPIDValues(), drive);
        }

        public Command setAngle(Rotation2d angle, Drive drive) {
                return Commands.run(() -> drive.setAngle(angle), drive);
        }

        // public static Command resetPigeon(Drive drive) {
        // return Commands.runOnce(() -> drive.resetGyro(), drive);
        // }

        /**
         * Field relative drive command using two joysticks (controlling linear and
         * angular velocities).
         */
        public Command joystickDrive(
                        Drive drive,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(speeds, true);
                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        speeds,
                                                        isFlipped
                                                                        ? drive.getRotation().plus(
                                                                                        new Rotation2d(Math.PI))
                                                                        : drive.getRotation());
                                },
                                drive);

        }

        public Command joystickDriveRobotRelative(
                        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
                return Commands.runEnd(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(speeds, true);
                                },
                                () -> drive.runVelocity(new ChassisSpeeds(), true),
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        public static class DriveToSuppliedPosition extends Command {
                private final Drive m_drive;
                private Pose2d m_target;
                private final Supplier<Pose2d> m_suppliedPose;
                private final ProfiledPIDController m_controllerX;
                private final ProfiledPIDController m_controllerY;
                private final ProfiledPIDController m_controllerTheta;
                private final Timer m_timer = new Timer();

                LoggedNetworkNumber kpTune = new LoggedNetworkNumber("tunes/translation kp", KP_XY);
                LoggedNetworkNumber kiTune = new LoggedNetworkNumber("tunes/translation ki", KI_XY);
                LoggedNetworkNumber kdTune = new LoggedNetworkNumber("tunes/translation kd", KD_XY);
                LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("tunes/translation max velocity",
                                MAX_VELOCETY_XY);
                LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("tunes/translation max acceleration",
                                MAX_ACCELERATION_XY);

                LoggedNetworkNumber kpThetaTune = new LoggedNetworkNumber("tunes/rotation kp", KP_OMEGA);
                LoggedNetworkNumber kiThetaTune = new LoggedNetworkNumber("tunes/rotation ki", KI_OMEGA);
                LoggedNetworkNumber kdThetaTune = new LoggedNetworkNumber("tunes/rotation kd", KD_OMEGA);
                LoggedNetworkNumber maxVelocityThetaTune = new LoggedNetworkNumber("tunes/rotation max velocity",
                                MAX_VELOCETY_OMEGA);
                LoggedNetworkNumber maxAccelerationThetaTune = new LoggedNetworkNumber(
                                "tunes/rotation max acceleration",
                                MAX_ACCELERATION_OMEGA);

                LoggedNetworkNumber translationTolerance = new LoggedNetworkNumber("tunes/translation tolerance",
                                TRANSLATION_TOLERANCE);
                LoggedNetworkNumber omegaTolerance = new LoggedNetworkNumber("tunes/rotation tolerance",
                                OMEGA_TOLERANCE);

                public DriveToSuppliedPosition(Drive drive, Supplier<Pose2d> target) {
                        m_drive = drive;
                        m_target = new Pose2d();
                        m_suppliedPose = target;
                        m_controllerX = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerX.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerY = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerY.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerTheta = new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA));
                        m_controllerTheta.setTolerance(OMEGA_TOLERANCE);
                        m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
                        addRequirements(drive);
                }

                @Override
                public void initialize() {
                        m_timer.reset();
                        m_timer.start();
                        var currPose = m_drive.getPose();
                        ChassisSpeeds currS = ChassisSpeeds.fromRobotRelativeSpeeds(m_drive.getChassisSpeeds(),
                                        currPose.getRotation());

                        m_target = m_suppliedPose.get();

                        m_controllerX.reset(currPose.getX(), currS.vxMetersPerSecond);
                        m_controllerY.reset(currPose.getY(), currS.vyMetersPerSecond);
                        m_controllerTheta.reset(currPose.getRotation().getRadians(), currS.omegaRadiansPerSecond);
                }

                @Override
                public void execute() {
                        var pose = m_drive.getPose();
                        var chassisSpeeds = new ChassisSpeeds(
                                        m_controllerX.calculate(pose.getTranslation().getX(),
                                                        m_target.getTranslation().getX()),
                                        m_controllerY.calculate(pose.getTranslation().getY(),
                                                        m_target.getTranslation().getY()),
                                        m_controllerTheta.calculate(pose.getRotation().getRadians(),
                                                        m_target.getRotation().getRadians()));

                        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
                                        pose.getRotation().unaryMinus());
                        Logger.recordOutput("Requested speeds", chassisSpeeds);
                        Logger.recordOutput("error x", m_target.getTranslation().getX() - pose.getTranslation().getX());
                        Logger.recordOutput("error y", m_target.getTranslation().getY() - pose.getTranslation().getY());
                        m_drive.runVelocity(chassisSpeeds, false);
                }

                @Override
                public boolean isFinished() {
                        return (m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerTheta.atGoal());
                }
        }
}