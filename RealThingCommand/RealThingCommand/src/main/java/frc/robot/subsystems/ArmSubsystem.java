package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleBoard9638;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

//DigitalInput toplimitSwitch = new DigitalInput(Constants.ArmConstants.ARM_UPPER_LIMIT_ID);
//DigitalInput bottomlimitSwitch = new DigitalInput(Constants.ArmConstants.ARM_LOWER_LIMIT_ID);
DigitalInput limitSwitchMag = new DigitalInput(Constants.ArmConstants.ARM_LOWER_LIMIT_MAG_ID);
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    public static String lastArmDirection; 
    /**
     * This subsytem that controls the arm.
     */
    public ArmSubsystem () {

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
    }

    @Override
    public void periodic() {
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runArm(double speed){
    
    ShuffleBoard9638.addDouble("Encoders", "Arm Encoder", armEncoder.getPosition() );
    armMotor.set(speed);
    
    }
    public void setArmDirection( String direction )
    {
      ShuffleBoard9638.addString("Arm Direction", direction);
      lastArmDirection = direction;
    }
    public boolean can_we_go_up()
    {
      if( lastArmDirection.equalsIgnoreCase(Constants.ArmConstants.ARM_UP_DIRECTION_STRING))
      {
        if( limitSwitchMag.get())
        {  
          return false;
        }
      }
      return true;

    }
    public boolean can_we_go_down()
    {
      if( lastArmDirection.equalsIgnoreCase(Constants.ArmConstants.ARM_DOWN_DIRECTION_STRING))
      {
        if( limitSwitchMag.get())
        {  
          return false;
        }
      }
      return true;

    }
}