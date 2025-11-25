// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;



public class DriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDrive drive;
  private boolean reverseRotation;
  private boolean reverseFront;
  private boolean speedToggle;

  private final SysIdRoutine sysIdRoutine;
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);


  /**
   * The subsystem used to drive the robot.
   */
  public DriveSubsystem() {
    // create brushed motors for drive
    
    reverseRotation = true;
    reverseFront = true;
    speedToggle = false;

    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    SendableRegistry.addChild(drive, leftLeader);
    SendableRegistry.addChild(drive, rightLeader);
    
    // Set CAN timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(DriveConstants.DRIVE_MOTOR_VOLTAGE_COMP);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    SendableRegistry.addChild(drive, leftEncoder);
    SendableRegistry.addChild(drive, rightEncoder);


    // Yikes...this is scary...
    sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
              leftLeader.setVoltage(voltage);
              rightLeader.setVoltage(voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
              // Record a frame for the left motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(leftEncoder.getPosition(), Meters))
                   .linearVelocity(m_velocity.mut_replace(leftEncoder.getVelocity(), MetersPerSecond));
              // Record a frame for the right motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(rightEncoder.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(rightEncoder.getVelocity(), MetersPerSecond)
                          );
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));



  }
  
  @Override  
  public void periodic() {
  }
  /**
   *  Use this to control your drive train, with one axis of the controller moving the robot
   *  forwards and backwards with the other axis turning the robot.
   * 
   *  Additionally if squared is true, it will square your controller inputs,
   *  for instance pushing forwards on the control stick will yield
   *  (0.5 * 0.5) = .25 or 25% power to the drivetrain.
   * 
   * @param xSpeed the speed forwards to back
   * @param zRotation the speed to turn at
   * @param squared do you square the inputs from the controller
   */
  public void driveArcade(double xSpeed, double zRotation, boolean squared) {
    double zRotationToUse;
    zRotationToUse = zRotation *.7;
    xSpeed = xSpeed *.8;
    if (reverseFront == true)
    {
      xSpeed = xSpeed * -1;
    }
    if (reverseRotation == true)
    {
      zRotationToUse = zRotation * -1;
    }
    
    double deadband;
    deadband = zRotation;
    deadband = Math.abs(deadband); 
    if (deadband < .2)
    {
         zRotationToUse = 0;
    }
    
    if ( speedToggle == true )
    {
      xSpeed = xSpeed *  DriveConstants.SLOW_MODE_MOVE;
      zRotationToUse = zRotationToUse * DriveConstants.SLOW_MODE_TURN;
    }
    else
    {
      zRotationToUse = zRotationToUse * Constants.DriveConstants.TURN_MULTIPLIER;
    }
    drive.arcadeDrive( xSpeed, zRotationToUse, squared);
  }

  




  
  /**
   * side of the drivetrain and the other stick controlling the other.
   * 
   * @param leftSpeed speed to drive the left side of the robot at
   * @param rightSpeed speed to drive the right side of the robot at
   * @param squared do you square the inputs from the controller 
   */
  public void driveTank(double leftSpeed, double rightSpeed, boolean squared){
  double leftSpeedToUse;
  double rightSpeedToUse;
  leftSpeedToUse = leftSpeed;
  rightSpeedToUse = rightSpeed;
  
    
    
    leftSpeedToUse = leftSpeedToUse *.8;
    rightSpeedToUse = rightSpeedToUse *.8;
    if (reverseFront == true)
    {
        leftSpeedToUse = -leftSpeedToUse;
        rightSpeedToUse = -rightSpeedToUse;
    }
    drive.tankDrive(leftSpeedToUse, rightSpeedToUse, squared);
  
  
  
  
  
  
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void reverseRotation()
  {
    if( reverseRotation == true)
      reverseRotation = false;
    else
      reverseRotation = true; 
  } 
  public void reverseFront()
  {
    reverseRotation();
    if( reverseFront == true)
      reverseFront = false;
    else
      reverseFront = true; 
  }
  public void speedToggle()
  {
    if( speedToggle == true)
      speedToggle = false;
    else
      speedToggle = true;


  }
}