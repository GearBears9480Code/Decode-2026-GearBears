package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.VisionConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && (alliance.get() == Alliance.Red);
    
    PhotonCamera camOne = new PhotonCamera("Arducam 1");
    PhotonCamera camTwo = new PhotonCamera("Arducam 2");

    Pose3d robotPose;
    Optional<Pose3d> tagPose;
    PhotonTrackedTarget target;
    Transform3d robotPoseTagRelative;
    Matrix<N3, N1> curStdDevs;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Transform3d kRobotToCamTwo = new Transform3d(new Translation3d(Units.inchesToMeters(-10.5), Units.inchesToMeters(13.5), 0), new Rotation3d(0, Math.toRadians(25), Math.toRadians(180)));
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(4.75), Units.inchesToMeters(14.75), 0), new Rotation3d(0, Math.toRadians(25), Math.toRadians(90)));
    
    public final PhotonPoseEstimator poseEstimatorOne = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    public final PhotonPoseEstimator poseEstimatorTwo = new PhotonPoseEstimator(kTagLayout, kRobotToCamTwo);
    
    private boolean apriltagSeen1 = false;
    private boolean apriltagSeen2 = false;

    double centerOfHub;
    private SwerveSubsystem swervesub;
    private Pose2d posOne = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d posTwo = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d truePos = new Pose2d(0, 0, new Rotation2d(0));
    
    public VisionSubsystem(SwerveSubsystem swerve) {
        System.out.println(camOne);
        if (isRed) {
            centerOfHub = kTagLayout.getTagPose(10).get().getX() + (PhysicalConstants.hubWidth / 2);
        } else {
            centerOfHub = kTagLayout.getTagPose(26).get().getX() + (PhysicalConstants.hubWidth / 2);
        }
        
        swervesub = swerve;

        SmartDashboard.putBoolean("cam1", false);
        SmartDashboard.putBoolean("cam2", false);

        SmartDashboard.putNumber("visPosition/angle", 0);
        SmartDashboard.putNumber("visPosition/x", 0);
        SmartDashboard.putNumber("visPosition/y", 0);
    }

    public double getData(boolean validTarget, PhotonPipelineResult output) {
        if (validTarget) {
            target = output.getBestTarget();
            if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                robotPoseTagRelative = target.getBestCameraToTarget();
                System.out.println(robotPoseTagRelative.getX());
            }
        }
 
        return robotPoseTagRelative.getX();
    }

    public Pose2d getVisionPose() {
        return swervesub.getPose();
    }

    public void periodic() {
        getVisionMeasurement(camOne, poseEstimatorOne, true);
        getVisionMeasurement(camTwo, poseEstimatorTwo, false);

        SmartDashboard.putNumber("visPosition/angle", truePos.getRotation().getDegrees());
        SmartDashboard.putNumber("visPosition/x", truePos.getX());
        SmartDashboard.putNumber("visPosition/y", truePos.getY());

        // var result = camOne.getLatestResult();
        // getData(result.hasTargets(), result);
    }

    public void getVisionMeasurement(PhotonCamera camera, PhotonPoseEstimator poseEstimator, boolean cam) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        var results = camera.getAllUnreadResults();
        for (var output : results) {
            visionEst = poseEstimator.estimateCoprocMultiTagPose(output);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(output);
            }

            updateEstimationStdDevs(visionEst, output.getTargets(), poseEstimator, cam);

            visionEst.ifPresent(
                est -> {
                    var estStdDev = getStdDev();
                    swervesub.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), Timer.getFPGATimestamp(), estStdDev);
                }
            );
        }
    }

    public void getAvgPose() {
        truePos = new Pose2d((posOne.getX() + posTwo.getX()) / 2, (posOne.getY() + posTwo.getY()) / 2, new Rotation2d((posOne.getRotation().getRadians() + posTwo.getRotation().getRadians()) / 2));
    }

    public Matrix<N2, N1> getPointFieldOriented(double xMeters, double yMeters) {
        // get the robot oriented point matrix
        Matrix<N2, N1> point = VecBuilder.fill(xMeters, yMeters);
        // rotate the point using rotation matrix
        Pose2d swervPose2d = swervesub.getPose();
        double theta = swervPose2d.getRotation().getRadians();
        double x = swervPose2d.getX();
        double y = swervPose2d.getY();
        point = MatBuilder.fill(N2.instance, N2.instance, Math.cos(theta), -Math.sin(theta), Math.sin(theta), Math.cos(theta)).times(point);
        point = point.plus(VecBuilder.fill(x, y));
        return point;
    }

    private void updateShuffleboardValue(boolean cam, boolean value) {
        if (cam) {
            apriltagSeen1 = value;
            SmartDashboard.putBoolean("cam1", value);
        } else {
            apriltagSeen2 = value;
            SmartDashboard.putBoolean("cam2", value);
        }
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator, boolean cam) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
            updateShuffleboardValue(cam, false);

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            updateShuffleboardValue(cam, true);
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }
            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            updateShuffleboardValue(cam, false);
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {estStdDevs = VisionConstants.kMultiTagStdDevs;}
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    {estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);}
                else {estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));}
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getStdDev() {
        return curStdDevs;
    }
}