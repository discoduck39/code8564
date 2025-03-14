// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechanismConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Commands.MoveElevatorToSetpoint;
import frc.robot.Commands.MoveFlipperToSetpoint;
import frc.robot.subsystems.ScoreMotorSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
       

        
        
        
    }

    private void configureBindings() {
        final CommandXboxController OXboxController = new CommandXboxController(1);
        final CommandXboxController DXboxController = new CommandXboxController(0);
        final ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
        final MechanismSubsystem s_MechanismSubsystem = new MechanismSubsystem();
        final ScoreMotorSubsystem s_MotorSubsystem = new ScoreMotorSubsystem();
    
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-DXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        DXboxController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        DXboxController.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DXboxController.getLeftY(), -DXboxController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DXboxController.back().and(DXboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DXboxController.back().and(DXboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DXboxController.start().and(DXboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DXboxController.start().and(DXboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        DXboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        DXboxController.a()
        

        drivetrain.registerTelemetry(logger::telemeterize);

         // operater/Driver controllers for the elevator, flipper, scorer//
        

         OXboxController.rightBumper().onTrue(
            new SequentialCommandGroup(
                new MoveFlipperToSetpoint(s_MechanismSubsystem, MechanismConstants.kalgeelevel1)));
        OXboxController.leftTrigger().onTrue(
            new SequentialCommandGroup(
                new MoveFlipperToSetpoint(s_MechanismSubsystem, MechanismConstants.kLevel2)));
                OXboxController.rightTrigger().onTrue(
            new SequentialCommandGroup(
                new MoveFlipperToSetpoint(s_MechanismSubsystem, MechanismConstants.kLevel3)));
                DXboxController.rightBumper().onTrue(
                    new SequentialCommandGroup(
                        new MoveFlipperToSetpoint(s_MechanismSubsystem, MechanismConstants.khome)));
                DXboxController.a().onTrue(
                    new SequentialCommandGroup(
                        new MoveFlipperToSetpoint(s_MechanismSubsystem, MechanismConstants.kLevel4)));
            OXboxController.leftBumper().onTrue(
                new SequentialCommandGroup(
                    new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel0)));
                    OXboxController.x().onTrue(
                new SequentialCommandGroup(
                    new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel1)));
                    OXboxController.y().onTrue(
                new SequentialCommandGroup(
                    new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel2)));
                    OXboxController.b().onTrue(
                new SequentialCommandGroup(
                    new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel3)));
                    OXboxController.a().onTrue(
                new SequentialCommandGroup(
                    new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel4)));
                
                

                    

                    

    
    
                    
       
                    

        

        
                    
        



    }
    


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

