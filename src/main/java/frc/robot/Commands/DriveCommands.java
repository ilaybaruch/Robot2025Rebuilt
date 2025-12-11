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
}