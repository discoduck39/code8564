// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveFlipperToSetpoint extends Command {

    private final MechanismSubsystem s_MechanismSubsystem;
    private final double Setpoint;

    /** Creates a new MoveElevatorToSetpoint. */
    public MoveFlipperToSetpoint(MechanismSubsystem s_MechanismSubsystem, double Setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.s_MechanismSubsystem = s_MechanismSubsystem;
        this.Setpoint = Setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Runs the elevator using Max Motion to the desired Setpoint specified in
        // Constants
        s_MechanismSubsystem.moveFlipperMOTOR3ToPostion(Setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // This is the end condition for the elevator subsystem,
        // The elevator will run until its encoder position is within 2 rotations of the
        // desired setpoint
        return Math.abs(s_MechanismSubsystem.getFlipperMOTOR3() - Setpoint) < 2.0;
    }
}