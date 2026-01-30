// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for interfacing with a Limelight camera via NetworkTables.
 * Provides methods to read AprilTag data including target angle.
 */
public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry tvEntry;
    private final NetworkTableEntry tidEntry;
    private final NetworkTableEntry priorityIdEntry;

    /**
     * Creates a new LimelightSubsystem with the default Limelight name.
     */
    public LimelightSubsystem() {
        this("limelight");
    }

    /**
     * Creates a new LimelightSubsystem with a custom Limelight name.
     *
     * @param limelightName The name of the Limelight (e.g., "limelight-front")
     */
    public LimelightSubsystem(String limelightName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        tvEntry = limelightTable.getEntry("tv");
        tidEntry = limelightTable.getEntry("tid");
        priorityIdEntry = limelightTable.getEntry("priorityid");
    }

    /**
     * Gets the horizontal angle offset to the target (tx).
     *
     * @return Horizontal offset from crosshair to target in degrees (-29.8 to 29.8)
     */
    public double getHorizontalAngle() {
        return txEntry.getDouble(0.0);
    }

    /**
     * Gets the vertical angle offset to the target (ty).
     *
     * @return Vertical offset from crosshair to target in degrees (-24.85 to 24.85)
     */
    public double getVerticalAngle() {
        return tyEntry.getDouble(0.0);
    }

    /**
     * Checks if the Limelight has a valid target.
     *
     * @return true if there is a valid target, false otherwise
     */
    public boolean hasTarget() {
        return tvEntry.getDouble(0.0) == 1.0;
    }

    /**
     * Gets the ID of the currently tracked AprilTag.
     *
     * @return The AprilTag ID, or -1 if no tag is detected
     */
    public int getTargetTagId() {
        return (int) tidEntry.getDouble(-1);
    }

    /**
     * Sets the priority AprilTag ID to track.
     * The Limelight will prioritize tracking this tag over others.
     *
     * @param tagId The AprilTag ID to prioritize
     */
    public void setPriorityTagId(int tagId) {
        priorityIdEntry.setNumber(tagId);
    }

    /**
     * Gets the horizontal angle to a specific AprilTag.
     * Returns the angle only if the currently tracked tag matches the requested ID.
     *
     * @param tagId The AprilTag ID to get the angle for
     * @return The horizontal angle in degrees, or Double.NaN if the tag is not visible
     */
    public double getAngleToTag(int tagId) {
        if (hasTarget() && getTargetTagId() == tagId) {
            return getHorizontalAngle();
        }
        return Double.NaN;
    }

    /**
     * Checks if a specific AprilTag is currently visible.
     *
     * @param tagId The AprilTag ID to check for
     * @return true if the specified tag is visible, false otherwise
     */
    public boolean isTagVisible(int tagId) {
        return hasTarget() && getTargetTagId() == tagId;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
