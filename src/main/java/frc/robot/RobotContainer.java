// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake_Com;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake_Sub;
import frc.robot.subsystems.Pivot_Sub;

public class RobotContainer {

  //Auto generated Swerve stuff (do not mess with unless otherwise told to)
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  //Define config files
  public Intake_Sub.Config intake_cfg = new Intake_Sub.Config("intake.toml");
  public Pivot_Sub.Config pivot_cfg = new Pivot_Sub.Config("pivot.toml");

  //Define subsystems
  private final Intake_Sub m_intake_Sub = new Intake_Sub(intake_cfg);
  private final Pivot_Sub m_pivot_Sub = new Pivot_Sub(pivot_cfg);
  
  

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);






    //Subsystem Control joystick.getRawAxis(0)
    joystick.a().whileTrue(new Intake_Com(m_intake_Sub, intake_cfg.intakeVoltage));
    joystick.b().whileTrue(new Intake_Com(m_intake_Sub, intake_cfg.outakeVoltage));


    joystick.rightBumper().onTrue(new InstantCommand(() -> m_pivot_Sub.DutyCycle(0.2)));
    joystick.rightBumper().onFalse(new InstantCommand(() -> m_pivot_Sub.DutyCycle(0)));
    joystick.leftBumper().onTrue(new InstantCommand(() -> m_pivot_Sub.DutyCycle(-0.2)));
    joystick.leftBumper().onFalse(new InstantCommand(() -> m_pivot_Sub.DutyCycle(0)));

    joystick.leftTrigger().onTrue(new InstantCommand(() -> m_pivot_Sub.PID(pivot_cfg.kHomePosition)));
    joystick.rightTrigger().onTrue(new InstantCommand(() -> m_pivot_Sub.PID(pivot_cfg.kExtendedPosition)));
    //m_intake_Sub.setDefaultCommand(new Intake_Com(m_intake_Sub, joystick.getRawAxis(1)));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
