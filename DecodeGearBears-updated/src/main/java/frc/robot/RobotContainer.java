// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActivateShooting;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ClientSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  public final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
  private final ClientSubsystem m_ClientSubsystem = new ClientSubsystem();
  final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(m_ClientSubsystem, m_SwerveSubsystem);
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_SwerveSubsystem);
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  private SwerveInputStream driveAngularVelocity;

  private ActivateShooting activateShooting = new ActivateShooting(m_HopperSubsystem, m_ShooterSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_mechanismController =
      new CommandXboxController(OperatorConstants.kMechanismControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureSwerveInputs();
    configureBindings();

    NamedCommands.registerCommand("IntakeToggle", new InstantCommand(() -> m_IntakeSubsystem.pid.togglePID()));
    NamedCommands.registerCommand("VacToggle", new InstantCommand(() -> m_IntakeSubsystem.toggleVacume()));
    NamedCommands.registerCommand("StartHopper", activateShooting);
    NamedCommands.registerCommand("StopHopper", new InstantCommand(() -> activateShooting.stop()));
    NamedCommands.registerCommand("ToggleShoot", new InstantCommand(() -> activateShooting.toggleFlywheel()));
    NamedCommands.registerCommand("ToggleVision", (new InstantCommand(() -> m_ShooterSubsystem.pid.manualToggle())));

    // SmartDashboard.putData("Shooter subsystem", m_ShooterSubsystem);
  }

  private void configureSwerveInputs() {
		driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getDrive(),
													() -> m_driverController.getLeftY() * -1,
													() -> m_driverController.getLeftX() * -1)
												.withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
												.deadband(0.1)
												.scaleTranslation(0.8)
												.allianceRelativeControl(true);
	}


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveAngularVelocityCommand = m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity);
		m_SwerveSubsystem.setDefaultCommand(driveAngularVelocityCommand);

    m_driverController.a().onTrue(new InstantCommand(() -> m_IntakeSubsystem.setArmRotation(1))).onFalse(new InstantCommand(() -> m_IntakeSubsystem.setArmRotation(0)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_IntakeSubsystem.setArmRotation(-1))).onFalse(new InstantCommand(() -> m_IntakeSubsystem.setArmRotation(0)));
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_ClimbSubsystem.setClimbVelocity(0.5))).onFalse(new InstantCommand(() -> m_ClimbSubsystem.setClimbVelocity(0)));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_ClimbSubsystem.setClimbVelocity(-0.5))).onFalse(new InstantCommand(() -> m_ClimbSubsystem.setClimbVelocity(0)));
    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_ClimbSubsystem.setArmVelocity(0.5))).onFalse(new InstantCommand(() -> m_ClimbSubsystem.setArmVelocity(0)));
    m_driverController.povRight().onTrue(new InstantCommand(() -> m_ClimbSubsystem.setArmVelocity(-0.5))).onFalse(new InstantCommand(() -> m_ClimbSubsystem.setArmVelocity(0)));

    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_IntakeSubsystem.toggleVacume()));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_IntakeSubsystem.pid.togglePID()));
    
    m_mechanismController.a().onTrue(new InstantCommand(() -> m_ShooterSubsystem.resetPositions()));
    m_mechanismController.x().onTrue(new InstantCommand(() -> activateShooting.toggleFlywheel()));
    m_mechanismController.rightBumper().onTrue(new InstantCommand(() -> m_ShooterSubsystem.pid.manualToggle()));
    m_mechanismController.rightTrigger(0.5).onTrue(activateShooting).onFalse(new InstantCommand(() -> activateShooting.stop()));

    m_mechanismController.povLeft().onTrue(new InstantCommand(() -> m_ShooterSubsystem.rotateTurret(0.1))).onFalse(new InstantCommand(() -> m_ShooterSubsystem.rotateTurret(0)));
    m_mechanismController.povRight().onTrue(new InstantCommand(() -> m_ShooterSubsystem.rotateTurret(-0.1))).onFalse(new InstantCommand(() -> m_ShooterSubsystem.rotateTurret(0)));
    m_mechanismController.povUp().onTrue(new InstantCommand(() -> m_ShooterSubsystem.rotateHood(0.3))).onFalse(new InstantCommand(() -> m_ShooterSubsystem.rotateHood(0)));
    m_mechanismController.povDown().onTrue(new InstantCommand(() -> m_ShooterSubsystem.rotateHood(-0.3))).onFalse(new InstantCommand(() -> m_ShooterSubsystem.rotateHood(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous skib idi
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_SwerveSubsystem.getAutonomousCommand("TestingAuto2");
  }
}
