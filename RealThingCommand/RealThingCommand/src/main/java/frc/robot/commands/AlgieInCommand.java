// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class AlgieInCommand extends Command {
  private final RollerSubsystem m_roller;
    private static  GenericEntry m_maxSpeed ;
  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public AlgieInCommand(RollerSubsystem roller) {
    m_roller = roller;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roller);
    if( m_maxSpeed == null)
    {
      m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Algie In",RollerConstants.ROLLER_ALGAE_IN)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", -1, "Max", 0))
        
            .withSize(2, 1)
            .getEntry();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // ShuffleBoard9638.addString("button", "algie in");

    m_roller.runRoller(m_maxSpeed.getDouble(RollerConstants.ROLLER_ALGAE_IN));
  }

  // Called once the command ends or is interrupted. This ensures the roller is
  // not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_roller.runRoller(-.05);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
