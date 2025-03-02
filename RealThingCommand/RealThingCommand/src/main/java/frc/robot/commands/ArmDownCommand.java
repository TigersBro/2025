// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An ArmDown command that uses an Arm subsystem. */
public class ArmDownCommand extends Command {
  private final ArmSubsystem m_arm;
  private static  GenericEntry m_maxSpeed ;
  /**
   * Powers the arm down, when finished passively holds the arm down.
   * 
   * We recommend that you use this to only move the arm into the paracord
   * and let the passive portion hold the arm down.
   *
   * @param arm The subsystem used by this command.
   */
  public ArmDownCommand(ArmSubsystem arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    if( m_maxSpeed == null)
    {
      m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Max Speed Arm Down", Constants.ArmConstants.ARM_SPEED_DOWN)
            .withWidget("Number Slider")
            // .withPosition(1, 1).withProperties(Map.of("min", 0, "max", 1))
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
    if (m_arm.can_we_go_down() )
    {
        m_arm.setArmDirection( ArmConstants.ARM_DOWN_DIRECTION_STRING);
      m_arm.runArm(m_maxSpeed.getDouble(ArmConstants.ARM_SPEED_DOWN));
    }
  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(ArmConstants.ARM_HOLD_DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
