    public Command getAutonomousCommand(String path) {
        return new PathPlannerAuto(path);
    }
package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    public double maxSpeedMeters = 3; 
    File directory = new File(Filesystem.getDeployDirectory(), "swerve"); // grabs the JSON packages for swerve
    private SwerveDrive swerveDrive;
    private RobotConfig config;
    public SwerveSubsystem() {    
        try { // you need a try catch statement because SwerveParser throws an exception that must be catched
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeedMeters);
        } catch(IOException e) {
            e.printStackTrace();
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                            this::getPose,
                            this::resetPose, 
                            this::getRobotChassisSpeeds, 
                            (speeds, feedforward) -> driveRobotRelative(speeds), 
                            new PPHolonomicDriveController(
                                new PIDConstants(5, 0, 0), 
                                new PIDConstants(5, 0, 0)
                            ), 
                            config, 
                            () -> {
                                Optional<Alliance> alliance = DriverStation.getAlliance();
                                if (alliance.isPresent()) {
                                    return alliance.get() == DriverStation.Alliance.Red;
                                }
                                return false;
                            }, 
                            this
                            );
    }

    public Command getDriveCommand(DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier xHeading, DoubleSupplier yHeading) {
        return run(() -> {
            Translation2d velocity = SwerveMath.scaleTranslation(new Translation2d(
                                                                                xTranslation.getAsDouble(),
                                                                                yTranslation.getAsDouble()
                                                                                ), 0.8);
            swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(velocity.getX(), velocity.getY(),
                                                                                        xHeading.getAsDouble(),
                                                                                        yHeading.getAsDouble(),
                                                                                        swerveDrive.getOdometryHeading().getRadians(),
                                                                                        swerveDrive.getMaximumChassisVelocity()));
        });
    }

    public Command driveFieldOriented(ChassisSpeeds velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity);
        });
    }

    public void driveRobotRelative(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public SwerveDrive getDrive() {
        return swerveDrive;
    }

    public void resetPose(Pose2d initialHolonomicPos) {
        swerveDrive.resetOdometry(initialHolonomicPos);
    }

}
