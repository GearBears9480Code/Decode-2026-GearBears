package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClientSubsystem extends SubsystemBase {
    DoubleArraySubscriber rawFiducials;
    
    double[] position = new double[3];
    int tagID;

    public ClientSubsystem() {
        NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
        rawFiducials = limelightData.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    }

    private void getAprilTag() {
        double[] aprilTagData = rawFiducials.get();
        System.out.print("Apriltag Data: ");
        for (double x : aprilTagData) {
            System.out.print(x + " ");
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
