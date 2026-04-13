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
    double yaw = 0;
    int tagID;

    public ClientSubsystem() {
        NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
        int[] redCrucialID = {8, 9, 10, 11};
        int[] blueCrucialID = {24, 25, 26, 27};
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        LimelightHelpers.SetFiducialIDFiltersOverride("", isRed ? redCrucialID : blueCrucialID);
        targetPose = limelightData.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    }

    private void getAprilTag() {
        try {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        
            tagID = fiducials[0].id;
            double[] aprilTagData = targetPose.get();

            position[0] = aprilTagData[0]; position[1] = aprilTagData[1]; position[2] = aprilTagData[2]; // set the current position
            yaw = aprilTagData[4];
        } catch (IndexOutOfBoundsException e) {
            tagID = 0;
            position[0] = 0; position[1] = 0; position[2] = 0;
            yaw = 0;
        }
    }

    public double[] getPose() {
        return position;
    }

    public double getDistance() {
        return Math.sqrt((position[0]*position[0]) + (position[2]*position[2]));
    }

    public int aprilTagID() {
        return tagID;
    }

    public void periodic() {
        getAprilTag();
    }

    public double[] rawOffsets(int ID) {
        double offsets[] = new double[2];

        switch (ID) {
            case 8:
                offsets[0] = 23.5;
                offsets[1] = 14;
            case 24:
                offsets[0] = 23.5;
                offsets[1] = 14;
            case 9:
                offsets[0] = 0;
                offsets[1] = 23.5;
            case 25:
                offsets[0] = 0;
                offsets[1] = 23.5;
            case 10:
                offsets[0] = -14;
                offsets[1] = 23.5;
            case 26:
                offsets[0] = -14;
                offsets[1] = 23.5;
            case 11:
                offsets[0] = -23.5;
                offsets[1] = 0;
            case 27:
                offsets[0] = -23.5;
                offsets[1] = 0;
            default:
                break;
        }

        return offsets;
    }

    public double[] getActualOffset(double[] offset, double angle) {
        double x = offset[0]; double y = offset[1];
        double a = Math.toRadians(angle);

        double xPrime = x * Math.cos(a) - y * Math.sin(a);
        double yPrime = y * Math.cos(a) + x * Math.sin(a);

        double[] ret = {xPrime, yPrime};
        return ret;
    }

    public double getDesiredAngle(boolean includeOffset) {
        double[] offset = getActualOffset(rawOffsets(tagID), yaw);
        double xLeg;
        double yLeg;

        if (includeOffset) {
            xLeg = position[0] - offset[0];
            yLeg = position[2] - offset[1];
        }
        else {
            xLeg = position[0];
            yLeg = position[2];
        }

        return Math.toDegrees(Math.atan2(xLeg, yLeg));
    }
}
