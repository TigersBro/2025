package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class ArmSubsystem extends SubsystemBase {

  // DigitalInput toplimitSwitch = new
  // DigitalInput(Constants.ArmConstants.ARM_UPPER_LIMIT_ID);
  // DigitalInput bottomlimitSwitch = new
  // DigitalInput(Constants.ArmConstants.ARM_LOWER_LIMIT_ID);
  DigitalInput limitSwitchMag = new DigitalInput(Constants.ArmConstants.ARM_LOWER_LIMIT_MAG_ID);
  private final SparkMax armMotor;
  private final RelativeEncoder armEncoder;
  public static String lastArmDirection = "NOTSET";



/*  This is for the sysid routine */
  private final SysIdRoutine sysIdRoutine;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  //private final NetworkTableEntry arm_position = Shuffleboard.getTab("Test").add("Arm Position",0).getEntry();
    

  /**
   * This subsytem that controls the arm.
   */
  public ArmSubsystem() {

   // Set up the arm motor as a brushed motor
    armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    armMotor.setCANTimeout(250);

    // Create and apply configuration for arm motor. Voltage compensation helps
    // the arm behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the arm stalls.
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.voltageCompensation(10);
    armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    armConfig.idleMode(IdleMode.kBrake);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder = armMotor.getEncoder();
    
    
    // Yikes...this is scary...
    sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
              armMotor.setVoltage(voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
              // Record a frame for the left motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          armMotor.get() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(armEncoder.getPosition(), Meters))
                   .linearVelocity(m_velocity.mut_replace(armEncoder.getVelocity(), MetersPerSecond));
              // Record a frame for the right motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
             
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));
  }

  @Override
  public void periodic() {
  }

  /**
   * This is a method that makes the arm move at your desired speed
   * Positive values make it spin forward and negative values spin it in reverse
   * 
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runArm(double speed) {

    armMotor.set(speed);
    

  }

  public void setArmDirection(String direction) {
    lastArmDirection = direction;
  }

  public boolean can_we_go_up() {
    if (lastArmDirection.equalsIgnoreCase(Constants.ArmConstants.ARM_UP_DIRECTION_STRING)) {
      if (limitSwitchMag.get()) {
        return false;
      }
    }
    return true;

  }

  public boolean can_we_go_down() {
    if (lastArmDirection.equalsIgnoreCase(Constants.ArmConstants.ARM_DOWN_DIRECTION_STRING)) {
      if (limitSwitchMag.get()) {
        return false;
      }
    }
    return true;

  }

  /**
   * Returns a command that will execute a quasistatic test in the given
   * direction.
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
}