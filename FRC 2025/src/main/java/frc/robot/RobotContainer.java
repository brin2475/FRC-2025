package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
   private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton resetModules = new JoystickButton(driver, XboxController.Button.kX.value); 
    
    private final JoystickButton start = new JoystickButton(driver, 1);
    private final JoystickButton stop = new JoystickButton(driver, 2);
    

    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); 
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value); 
    private final JoystickButton y = new JoystickButton(driver, XboxController.Button.kY.value); 
    private final JoystickButton a = new JoystickButton(driver, XboxController.Button.kA.value); 
    private final JoystickButton b = new JoystickButton(driver, XboxController.Button.kB.value); 
    private final JoystickButton x = new JoystickButton(driver, XboxController.Button.kX.value); 



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final sysidSubsystem m_SysidSubsystem = new sysidSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()
                )
        );

        // Configure the button bindings
        configureButtonBindings();


        

      
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        resetModules.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())); 


        start.onTrue(new InstantCommand(() -> m_SysidSubsystem.startLogging())); 
        stop.onTrue(new InstantCommand(() -> m_SysidSubsystem.stopLogging())); 
       
        leftBumper.onTrue(Commands.runOnce(SignalLogger::start));
        rightBumper.onTrue(Commands.runOnce(SignalLogger::stop));

        /*
        * Joystick Y = quasistatic forward
        * Joystick A = quasistatic reverse
        * Joystick B = dynamic forward
        * Joystick X = dyanmic reverse
        */
        y.whileTrue(m_SysidSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        a.whileTrue(m_SysidSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        b.whileTrue(m_SysidSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        x.whileTrue(m_SysidSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}