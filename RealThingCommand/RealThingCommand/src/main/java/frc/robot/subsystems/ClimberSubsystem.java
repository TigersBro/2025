package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climbMotorright;
  private final SparkMax climbMotorleft;
    private final RelativeEncoder climbEncoder;
    private final SysIdRoutine sysIdRoutine;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    /**
     * This subsytem that controls the climber.
     */
    public ClimberSubsystem () {

    // Set up the climb motor as a brushless motor
    climbMotorleft = new SparkMax(ClimberConstants.CLIMBER_leftwinch_MOTOR_ID, MotorType.kBrushed);
    climbMotorright = new SparkMax(ClimberConstants.CLIMBER_rightwinch_MOTOR_ID, MotorType.kBrushed);
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    climbMotorleft.setCANTimeout(250);
    climbMotorright.setCANTimeout(250);

    // Create and apply configuration for climb motor. Voltage compensation helps
    // the climb behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the climb stalls.
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
    climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotorleft.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbConfig.follow(climbMotorleft);
    climbMotorright.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbEncoder = climbMotorleft.getEncoder();

    

    // Yikes...this is scary...
    //yup
    sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
              climbMotorleft.setVoltage(voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
              // Record a frame for the left motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          climbMotorleft.get() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(climbEncoder.getPosition(), Meters))
                   .linearVelocity(m_velocity.mut_replace(climbEncoder.getVelocity(), MetersPerSecond));
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
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climbMotorleft.set(speed);
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
}