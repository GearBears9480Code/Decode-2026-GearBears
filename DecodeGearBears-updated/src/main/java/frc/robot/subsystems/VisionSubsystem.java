package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.VisionConstants;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.ironmaple.simulation.Goal.RotationChecker;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionSubsystem extends SubsystemBase {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && (alliance.get() == Alliance.Red);
    
    PhotonCamera camOne = new PhotonCamera("Arducam One");
    PhotonCamera camTwo = new PhotonCamera("Arducam Two");

    Pose3d robotPose;
    Optional<Pose3d> tagPose;
    PhotonTrackedTarget target;
    Transform3d robotPoseTagRelative;
    Matrix<N3, N1> curStdDevs;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(10), 0), new Rotation3d(0, Math.toRadians(-50), Math.toRadians(180)));
    public final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    double centerOfHub;
    private SwerveSubsystem swervesub;
    
    public VisionSubsystem(SwerveSubsystem swerve) {
        if (isRed) {
            centerOfHub = kTagLayout.getTagPose(10).get().getX() + (PhysicalConstants.hubWidth / 2);
        } else {
            centerOfHub = kTagLayout.getTagPose(26).get().getX() + (PhysicalConstants.hubWidth / 2);
        }
        
        swervesub = swerve;

        System.out.println("Center of the Hub: " + centerOfHub);
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

    public void periodic() {
        getVisionMeasurement(camOne);
        // getVisionMeasurement(camTwo, VisionConstants.camTwoPose);

        // var result = camOne.getLatestResult();
        // getData(result.hasTargets(), result);
    }

    public void getVisionMeasurement(PhotonCamera camera) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        var results = camera.getAllUnreadResults();
        for (var output : results) {
            visionEst = poseEstimator.estimateCoprocMultiTagPose(output);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(output);
            }
            if (!results.isEmpty()) {

            }

            updateEstimationStdDevs(visionEst, output.getTargets());

            visionEst.ifPresent(
                est -> {
                    var estStdDevs = getStdDev();

                    swervesub.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), Timer.getTimestamp(), estStdDevs);
                }
            );
        }
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
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