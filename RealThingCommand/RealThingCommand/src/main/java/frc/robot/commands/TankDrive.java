// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TankDrive extends Command {

  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;
  private final DriveSubsystem m_drive;
  private final BooleanSupplier m_squared;


  /** Creates a new TankDrive. */
  public TankDrive(DriveSubsystem driveSubsystem, 
      DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, BooleanSupplier squareInputs) {
        m_leftSpeed = leftSpeed;
        m_rightSpeed = rightSpeed;
        m_drive = driveSubsystem;
        m_squared = squareInputs;
        addRequirements(m_drive);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //double modifier;
    //modifier = m_zRotation.getAsDouble() * -1;
    m_drive.driveTank(m_leftSpeed.getAsDouble(),m_rightSpeed.getAsDouble(), m_squared.getAsBoolean());
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
