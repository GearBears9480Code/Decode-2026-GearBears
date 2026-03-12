package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionSubsystem extends SubsystemBase {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    
    PhotonCamera camOne = new PhotonCamera("Arducam One");
    PhotonCamera camTwo = new PhotonCamera("Arducam Two");
    Pose3d robotPose3d;
    PhotonTrackedTarget target;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    private Matrix<N3, N1> currentStdDevs;

    double centerOfHub;
    private ShooterSubsystem shooter;
    
    public VisionSubsystem(ShooterSubsystem shoot) {
        shooter = shoot;
        if (isRed) {
            centerOfHub = kTagLayout.getTagPose(10).get().getX() + (PhysicalConstants.hubWidth / 2);
        } else {
            centerOfHub = kTagLayout.getTagPose(26).get().getX() + (PhysicalConstants.hubWidth / 2);
        }

        System.out.println("Center of the Hub: " + centerOfHub);
    }

    public double getData(boolean validTarget, PhotonPipelineResult output) {
        if (validTarget) {
            target = output.getBestTarget();
            if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), kTagLayout.getTagPose(target.getFiducialId()).get(), kRobotToCam);
            }
        }

        return target.getYaw();
    }
    
    /*
    public Optional<EstimatedRobotPose> getFieldRelativePose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonPipelineResult> unreadResults = camOne.getAllUnreadResults();

        for (var result : unreadResults) {
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
        }

        return visionEst;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    }

    private Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }
    */

    public void periodic() {
        PhotonPipelineResult camOneResult = camOne.getLatestResult();
        PhotonPipelineResult camTwoResult = camTwo.getLatestResult();

        boolean hasTargets = camOneResult.hasTargets();
        getData(hasTargets, camOneResult);
        shooter.rotateTurret(getData(hasTargets, camOneResult));
    }
}
