// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverController = new CommandPS5Controller(0);

    final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve"));

    private final frc.robot.subsystems.ShooterSubsystem m_shooter = new frc.robot.subsystems.ShooterSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> driverController.getLeftY() * -1,
                  () -> driverController.getLeftX() * -1)
          .withControllerRotationAxis(driverController::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                  driverController::getRightY)
          .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
          .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverController.getLeftY(),
                  () -> -driverController.getLeftX())
          .withControllerRotationAxis(() -> driverController.getRawAxis(
                  2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
          .withControllerHeadingAxis(() ->
                          Math.sin(
                                  driverController.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2),
                  () ->
                          Math.cos(
                                  driverController.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(
                  0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverController.getRightX(),
            () -> driverController.getRightY());

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);






    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
            .andThen(Commands.none()));

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
            .andThen(drivebase.driveForward().withTimeout(1)));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null ) {
      RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
      Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
      Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
      Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);


      if (RobotBase.isSimulation()) {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      } else {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }

      if (Robot.isSimulation()) {
          Pose2d target = new Pose2d(new Translation2d(1, 4),
                  Rotation2d.fromDegrees(90));
          //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
          driveDirectAngleKeyboard.driveToPose(() -> target,
                  new ProfiledPIDController(5,
                          0,
                          0,
                          new Constraints(5, 2)),
                  new ProfiledPIDController(5,
                          0,
                          0,
                          new Constraints(Units.degreesToRadians(360),
                                  Units.degreesToRadians(180))
                  ));
          driverController.button(10).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
          driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
          driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));


      }
      if (DriverStation.isTest()) {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

          driverController.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
          driverController.button(10).onTrue((Commands.runOnce(drivebase::zeroGyro)));
          driverController.button(11).whileTrue(drivebase.centerModulesCommand());
          driverController.L1().onTrue(Commands.none());
          driverController.R1().onTrue(Commands.none());
      } else {
          driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
          driverController.button(10).whileTrue(Commands.none());
          driverController.button(11).whileTrue(Commands.none());
          driverController.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
          driverController.R1().onTrue(Commands.none());
      }

      {
          operatorController.R1().whileTrue(
                  new StartEndCommand(
                          () -> m_shooter.spin(),
                          () -> m_shooter.stop(),
                          m_shooter
                  )
          );

          // לחיצה על L1: חילוץ כדור
          operatorController.L1().whileTrue(
                  new StartEndCommand(
                          () -> m_shooter.reverse(),
                          () -> m_shooter.stop(),
                          m_shooter
                  )
          );

      }
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
