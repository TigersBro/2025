// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AlgieAuto;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.SimpleCoralAuto;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
    UsbCamera camera = CameraServer.startAutomaticCapture();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
  // You can remove this if you wish to have a single driver, note that you
  // may have to change the binding for left bumper.
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final DriveSubsystem m_drive = new DriveSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  public final SimpleCoralAuto m_simpleCoralAuto = new SimpleCoralAuto(m_drive, m_roller, m_arm);
  public final DriveForwardAuto m_driveForwardAuto = new DriveForwardAuto(m_drive);
  public final AlgieAuto m_algieAuto = new AlgieAuto(m_drive, m_roller, m_arm);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    SmartDashboard.putData("Drive",m_drive);
    SmartDashboard.putData("Arm", m_arm);
    SmartDashboard.putData("Climber", m_climber); 
    SmartDashboard.putData("Roller", m_roller);

    Shuffleboard.getTab("Commands").add("Algie In",new AlgieInCommand(m_roller));
    Shuffleboard.getTab("Commands").add("Algie Out",new AlgieOutCommand(m_roller));
    Shuffleboard.getTab("Commands").add("Arm Up",new ArmUpCommand(m_arm));
    Shuffleboard.getTab("Commands").add("Arm Down",new ArmDownCommand(m_arm));
    Shuffleboard.getTab("Commands").add("Climber Down",new ClimberDownCommand(m_climber));
    Shuffleboard.getTab("Commands").add("Climber Up",new ClimberUpCommand(m_climber));
    Shuffleboard.getTab("Commands").add("Reverse Rotation",new InstantCommand( () -> m_drive.reverseRotation()));
    Shuffleboard.getTab("Commands").add("Reverse Front",new InstantCommand( () -> m_drive.reverseFront()));

    m_chooser.setDefaultOption("Drive Forward Auto",  m_driveForwardAuto);
    m_chooser.addOption("Algie Auto", m_algieAuto);
    m_chooser.addOption("Coral Auto",m_simpleCoralAuto);
    SmartDashboard.putData(m_chooser);


        // Log Shuffleboard events for command initialize, execute, finish, interrupt
//     CommandScheduler.getInstance()
//         .onCommandInitialize(
//             command ->
//                 Shuffleboard.addEventMarker(
//                     "Command initialized", command.getName(), EventImportance.kNormal));
//     CommandScheduler.getInstance()
//         .onCommandExecute(
//             command ->
//                 Shuffleboard.addEventMarker(
//                     "Command executed", command.getName(), EventImportance.kNormal));
//     CommandScheduler.getInstance()
//         .onCommandFinish(
//             command ->
//                 Shuffleboard.addEventMarker(
//                     "Command finished", command.getName(), EventImportance.kNormal));
//     CommandScheduler.getInstance()
//         .onCommandInterrupt(
//             command ->
//                 Shuffleboard.addEventMarker(
//                     "Command interrupted", command.getName(), EventImportance.kNormal));
   }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /**
     * Set the default command for the drive subsystem to an instance of the
     * DriveCommand with the values provided by the joystick axes on the driver
     * controller. The Y axis of the controller is inverted so that pushing the
     * stick away from you (a negative value) drives the robot forwards (a positive
     * value). Similarly for the X axis where we need to flip the value so the
     * joystick matches the WPILib convention of counter-clockwise positive
     */
    m_drive.setDefaultCommand(new DriveCommand(m_drive,
        
        () -> -m_driverController.getY(),
        () -> -m_driverController.getZ(),
        () -> true));

    /**
     * Holding the button 2 (thumb button) (or whatever button you assign) will multiply the
     * speed
     * by a decimal to limit the max speed of the robot ->
     * 1 (100%) from the controller * .9 = 90% of the max speed when held (we also
     * square it)
     * 
     * Slow mode is very valuable for line ups and the deep climb
     * 
     * When switching to single driver mode switch to the B button
     */
    m_driverController.button(2).whileTrue(new DriveCommand(m_drive,
        () -> -m_driverController.getY() * DriveConstants.SLOW_MODE_MOVE,
        () -> -m_driverController.getZ() * DriveConstants.SLOW_MODE_TURN,
        () -> true));

    m_operatorController.L2().whileTrue(new AlgieInCommand(m_roller));
    m_operatorController.R2().whileTrue(new AlgieOutCommand(m_roller));

    /**
     * The arm will be passively held up or down after this is used,
     * make sure not to run the arm too long or it may get upset!
     */
    m_operatorController.L1().whileTrue(new ArmUpCommand(m_arm));
    m_operatorController.R1().whileTrue(new ArmDownCommand(m_arm));

    m_operatorController.pov(0).whileTrue(new ClimberUpCommand(m_climber));
    m_operatorController.pov(180).whileTrue(new ClimberDownCommand(m_climber));

    
    m_driverController.button(DriveConstants.DRIVE_REVERSE_ROTATION_BUTTON_ID).toggleOnTrue(new InstantCommand( () -> m_drive.reverseRotation() ));
    m_driverController.button(DriveConstants.DRIVE_REVERSE_FRONT_BUTTON_ID).toggleOnTrue(new InstantCommand( () -> m_drive.reverseFront() ));
    m_driverController.button(ArmConstants.ARM_OVERRIDE_BUTTON).toggleOnTrue(new InstantCommand( () -> m_arm.set_limit_switch_bypass(true) ));


    // * Here we declare all of our operator commands, these commands could have
    // been
    // * written in a more compact manner but are left verbose so the intent is
    // clear.
    // */

    // // sysidstuff comment out when done.
    // m_operatorController
    //     .triangle()
    //     .and(m_operatorController.R1())
    //     .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_operatorController
    //     .circle()
    //     .and(m_operatorController.R1())
    //     .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_operatorController
    //     .cross()
    //     .and(m_operatorController.R1())
    //     .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_operatorController
    //     .square()
    //     .and(m_operatorController.R1())
    //     .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Control the shooter wheel with the left trigger
    // m_arm.setDefaultCommand(m_arm.runShooter(m_operatorController::getLeftTriggerAxis));

    ///////// ARM SYSID////////////
    // m_operatorController
    //     .triangle()
    //     .and(m_operatorController.L1())
    //     .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_operatorController
    //     .circle()
    //     .and(m_operatorController.L1())
    //     .whileTrue(m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_operatorController
    //     .cross()
    //     .and(m_operatorController.L1())
    //     .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_operatorController
    //     .square()
    //     .and(m_operatorController.L1())
    //     .whileTrue(m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
