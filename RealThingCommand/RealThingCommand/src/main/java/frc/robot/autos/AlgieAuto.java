package frc.robot.autos;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class AlgieAuto extends Command {
    private DriveSubsystem m_drive;
    private RollerSubsystem m_roller;
    private ArmSubsystem m_arm;
    private Timer timer;
    private double drive_seconds = 2.5;
    private double turn_seconds = drive_seconds + .95;
    private double score_seconds = turn_seconds + .75;
    private double kobe = score_seconds + 1;

    /**
     * This auto will have the robot drive forwards, stop, then drop the coral into L1
     * 
     * There are many ways to write autos, this style will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use two timer gates, after the robot has finished driving for the first 3.25 
     * seconds, it will exjest the coral for 4.5-3.25 = 1.25 seconds.
     * 
     * 
     * @param drive
     * @param roller
     * @param arm
     */
    public AlgieAuto(DriveSubsystem drive, RollerSubsystem roller, ArmSubsystem arm)
    {
        m_drive = drive;
        m_roller = roller;
        m_arm = arm;
        
        timer = new Timer();

        addRequirements(m_drive);
        addRequirements(m_roller);
        addRequirements(m_arm);

    //          Shuffleboard.getTab("Configuration")
    //         .add("Algie Auto", .85)
    //         .withWidget("Number Slider")
    //         // .withPosition(1, 1)
    //         .withProperties(Map.of("min", 0, "max", 1.5))
        
    //         .withSize(2, 1)
    //         .getEntry();
    // 
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    /**
     * We always want to hold the arm up duirng the auto to ensure the rollers
     */ 
    m_arm.runArm(ArmConstants.ARM_HOLD_UP);

    /**
     * While this timer is less than drive_seconds, the robot will obey the command inside
     * This could benefit from using the distance versus using a timer.
     */
    if(timer.get() < drive_seconds)
    {
        m_drive.driveArcade(Constants.DriveConstants.SUPER_SLOW_MODE_MOVE, 0.0,false);
    }
    /**
     * Once the timer is greater than drive_seconds but less than turn seconds,
     * we will turn the bot to point towards the score this could use the qr tag if we can figure that out.
     */
    else if(timer.get() > drive_seconds && timer.get() < turn_seconds)
    {
        m_drive.driveArcade(0.0, -Constants.DriveConstants.SUPER_SLOW_MODE_TURN,false);
    }
    //drive forward for a few moments to get the ball in scoring position
    else if (timer.get() > turn_seconds && timer.get() < score_seconds)
    {

      m_drive.driveArcade(Constants.DriveConstants.SUPER_SLOW_MODE_MOVE, 0, false);
    
    }
    else if (timer.get() > score_seconds && timer.get() < kobe)
    {

      m_drive.driveArcade(0, 0, false);
      m_roller.runRoller(Constants.RollerConstants.ROLLER_CORAL_OUT);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    m_drive.driveArcade(0.0, 0.0, false);
    m_roller.runRoller(0);
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= kobe;
  }
}
