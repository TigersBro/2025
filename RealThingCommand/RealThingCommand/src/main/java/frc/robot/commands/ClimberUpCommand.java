// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An ClimberUpCommand that uses a climb subsystem. */
public class ClimberUpCommand extends Command {
  private final ClimberSubsystem m_climber;
 public static GenericEntry m_maxSpeed ;
  /**
   * Runs the climber up, note that this can change 
   * based on how the winch is wound.
   *
   * @param climber The subsystem used by this command.
   */
  public ClimberUpCommand(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(climber);

    if (m_maxSpeed == null)  
    {
        m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Max Speed Climb Up", Constants.ClimberConstants.CLIMBER_SPEED_UP)
            .withWidget("Number Slider")
            // .withPosition(1, 1)
            .withProperties(Map.of("min", 0, "max", 1))
            .withSize(2, 1)
            .getEntry();
            
    }

    
      // m_maxSpeed =
      //     Shuffleboard.getTab("Configuration")
      //         .add("Max Speed Climb Up", Constants.ClimberConstants.CLIMBER_SPEED_UP)
      //         .withWidget("Number Slider")
      //         // .withPosition(1, 1)
      //         .withProperties(Map.of("min", 0, "max", 1))
      //         .withSize(2, 1)
      //         .getEntry();
      //       }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_climber.runClimber(m_maxSpeed.getDouble(ClimberConstants.CLIMBER_SPEED_UP));
  }

  // Called once the command ends or is interrupted.. Here we ensure the climber is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
