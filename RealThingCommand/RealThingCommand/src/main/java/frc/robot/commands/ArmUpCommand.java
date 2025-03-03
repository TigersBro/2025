// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An ArmUpCommand that uses an Arm subsystem. */
public class ArmUpCommand extends Command {
  private final ArmSubsystem m_arm;
  private static GenericEntry m_maxSpeed;
  private static GenericEntry m_holdArm;

  /**
   * Powers the arm up, when finished passively holds the arm up.
   * 
   * We recommend that you use this to only move the arm into the hardstop
   * and let the passive portion hold the arm up.
   *
   * @param arm The subsystem used by this command.
   */
  public ArmUpCommand(ArmSubsystem arm) 
  {
    m_arm = arm;
    addRequirements(arm);
    if (m_maxSpeed == null)
    {    
      m_maxSpeed
        =
            Shuffleboard.getTab("Configuration")
                .add("Max Speed Arm Up", ArmConstants.ARM_SPEED_UP)
                .withWidget("Number Slider")
                // .withPosition(1, 1)
                .withProperties(Map.of("min", -1, "max", 0))
        
                .withSize(2, 1)
                .getEntry();
    } 
    if (m_holdArm == null)
    {
      m_holdArm
      =
          Shuffleboard.getTab("Configuration")
              .add("Hold Arm Up", ArmConstants.ARM_HOLD_UP)
              .withWidget("Number Slider")
              // .withPosition(1, 1)
              .withProperties(Map.of("min", 0, "max", 1))
        
              .withSize(2, 1)
              .getEntry();
    }


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //ShuffleBoard9638.addString("button","arm up");
 //if (m_arm.can_we_go_up() )
    {
      m_arm.setArmDirection( ArmConstants.ARM_UP_DIRECTION_STRING);
      m_arm.runArm(m_maxSpeed.getDouble(ArmConstants.ARM_SPEED_UP));
    
    }
  }

  // Called once the command ends or is interrupted.
  // Here we run a command that will hold the arm up after to ensure the arm does
  // not drop due to gravity.
  @Override
  public void end(boolean interrupted) {
    
      m_arm.runArm(-.01);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
