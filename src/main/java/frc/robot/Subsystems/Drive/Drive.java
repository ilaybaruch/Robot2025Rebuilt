package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drive extends SubsystemBase {
    boolean goodVison = true;
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    // public static double ODOMETRY_FREQUENCY = 0.0;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    public Drive(GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO);
        modules[1] = new Module(frModuleIO);
        modules[2] = new Module(blModuleIO);
        modules[3] = new Module(brModuleIO);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // need to add odometry stuff
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds(), true);
    }

    public void runVelocity(ChassisSpeeds speeds, boolean isOpenLoop) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);// need to check how it works
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);// need to check how it works
                                                                                          // x2

        if (Math.abs(speeds.omegaRadiansPerSecond + speeds.vxMetersPerSecond + speeds.vyMetersPerSecond) < 0.01) { // stops
                                                                                                                   // modules
                                                                                                                   // from
                                                                                                                   // ticking
            for (int i = 0; i < 4; i++) {
                modules[i].stop();
            }
        } else {
            // Send setpoints to modules
            for (int i = 0; i < 4; i++) {
                modules[i].runSetPoint(setpointStates[i], isOpenLoop);
                ;
            }

        }

    }

    public void setAngle(Rotation2d angle) {
        for (int i = 0; i < 4; i++) {
            modules[i].setAngle(angle);
        }
    }

    public void setVelocity(double velocityRadPerSec) {
        for (int i = 0; i < 4; i++) {
            modules[i].setVelocity(velocityRadPerSec);
        }
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void setPIDValues() {
        for (int i = 0; i < 4; i++) {
            modules[i].driveTune();
        }
    }

}
