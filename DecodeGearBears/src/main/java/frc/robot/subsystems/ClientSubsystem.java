package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class ClientSubsystem extends SubsystemBase {
    DoubleArraySubscriber targetPose;
   
    double[] position = new double[3];
    int tagID;

    public ClientSubsystem() {
        NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
        int[] redCrucialID = {2, 11, 3, 4, 5, 8, 9, 10};
        int[] blueCrucialID = {19, 20, 21, 24, 25, 26, 27, 18};
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        LimelightHelpers.SetFiducialIDFiltersOverride("", isRed ? redCrucialID : blueCrucialID);
        targetPose = limelightData.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    }

    private void getAprilTag() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        double[] aprilTagData = targetPose.get();
        System.out.print("Apriltag Data: ");
        for (double x : aprilTagData) {
            System.out.print(x + " ");
        }

        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;
            System.out.println("Tag ID: " + id + "");
        }
        System.out.println();
    }

    public double[] getPose() {
        return position;
    }

    public int aprilTagID() {
        return tagID;
    }

    public void periodic() {
        getAprilTag();
    }
}
