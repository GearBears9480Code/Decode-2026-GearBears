// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * A command that reads the angle to a specific AprilTag from the Limelight.
 * The command will set the priority tag ID and wait until the tag is visible,
 * then return the angle via a callback consumer.
 */
public class GetAprilTagAngleCommand extends Command {
    private final LimelightSubsystem m_limelight;
    private final int m_tagId;
    private final Consumer<Double> m_angleConsumer;
    private double m_angle;
    private boolean m_foundTag;

    /**
     * Creates a new GetAprilTagAngleCommand.
     *
     * @param limelight The Limelight subsystem to use
     * @param tagId The AprilTag ID to find and measure angle to
     * @param angleConsumer A consumer that will receive the angle when found (in degrees)
     */
    public GetAprilTagAngleCommand(LimelightSubsystem limelight, int tagId, Consumer<Double> angleConsumer) {
        m_limelight = limelight;
        m_tagId = tagId;
        m_angleConsumer = angleConsumer;
        addRequirements(limelight);
    }

    /**
     * Creates a new GetAprilTagAngleCommand without a consumer.
     * Use getAngle() after the command completes to retrieve the result.
     *
     * @param limelight The Limelight subsystem to use
     * @param tagId The AprilTag ID to find and measure angle to
     */
    public GetAprilTagAngleCommand(LimelightSubsystem limelight, int tagId) {
        this(limelight, tagId, null);
    }

    @Override
    public void initialize() {
        m_foundTag = false;
        m_angle = Double.NaN;
        // Set the priority tag ID so the Limelight focuses on this tag
        m_limelight.setPriorityTagId(m_tagId);
    }

    @Override
    public void execute() {
        if (m_limelight.isTagVisible(m_tagId)) {
            m_angle = m_limelight.getHorizontalAngle();
            m_foundTag = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && m_foundTag && m_angleConsumer != null) {
            m_angleConsumer.accept(m_angle);
        }
    }

    @Override
    public boolean isFinished() {
        return m_foundTag;
    }

    /**
     * Gets the angle found by this command.
     * Should be called after the command has completed.
     *
     * @return The horizontal angle to the AprilTag in degrees, or Double.NaN if not found
     */
    public double getAngle() {
        return m_angle;
    }

    /**
     * Checks if the tag was found during command execution.
     *
     * @return true if the tag was found, false otherwise
     */
    public boolean wasTagFound() {
        return m_foundTag;
    }
}
