// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to remove (score or pass) Algae. */
public class AlgieOutCommand extends Command {
  private final RollerSubsystem m_roller;
  private static  GenericEntry m_maxSpeed ;
  /**
   * Rolls the Algae out of the intake.
   * We recommend not using this to score coral.
   *
   * @param roller The subsystem used by this command.
   */
  public AlgieOutCommand(RollerSubsystem roller) {
    m_roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roller);
        if( m_maxSpeed == null)
    {
      m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Algie Out", RollerConstants.ROLLER_ALGAE_OUT)
            .withWidget("Number Slider")
            // .withPosition(1, 1)
            .withProperties(Map.of("min", 0, "max", 1))
        
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
    //ShuffleBoard9638.addString("button", "algie out");

    m_roller.runRoller(m_maxSpeed.getDouble(RollerConstants.ROLLER_ALGAE_OUT));
  }

  // Called once the command ends or is interrupted. This ensures the roller is
  // not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_roller.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
