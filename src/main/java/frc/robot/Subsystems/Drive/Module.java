package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    // private final int index;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    Module(ModuleIO io/* , int index */) {
        this.io = io;
        // this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);

        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public void runSetPoint(SwerveModuleState state, boolean isOpenLoop) {

        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        if (isOpenLoop) {
            io.setDriveOpenLoop(state.speedMetersPerSecond / maxSpeedMetersPerSec * 12);
        } else {
            io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
        }
        io.setTurnPosition(state.angle);
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    public void runSteerCharacterization(double output) {
        io.setDriveOpenLoop(0);
        io.setTurnOpenLoop(output);
    }

    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    public void driveTune() {
        io.setPIDValues();
    }

    public void setAngle(Rotation2d angle) {
        io.setTurnPosition(angle);
    }

    public void setVelocity(double velocityRadPerSec) {
        io.setDriveVelocity(velocityRadPerSec);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadiusMeters;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    // public SwerveModulePosition[] getOdometryPositions() {
    // return odometryPositions;
    // }

    // public double[] getOdometryTimestamps() {
    // return inputs.odometryTimestamps;
    // }

}