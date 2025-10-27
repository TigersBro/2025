// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public class cannon4 extends Command {
  /** Creates a new cannon4. */
  private final Pneumatics m_Pneumatics;
  private boolean m_done; 

  private Timer m_timer = new Timer();


  public cannon4(Pneumatics pn) {
    m_Pneumatics = pn;
    m_done = false;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Pneumatics.fire4();

    m_timer.reset();
    m_timer.start();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Pneumatics.stop4();
    m_done = true;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(1)){
      m_done  = true;
    }
    else{
      m_done = false;
    }
    return m_done;
  }
}
