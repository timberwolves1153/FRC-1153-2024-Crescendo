// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.Auto.TestAuto;
import frc.robot.commands.RotateAndX;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.lib.util.AxisButton;
import frc.robot.subsystems.Collector;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final Swerve s_Swerve = new Swerve();
    private final Winch elevator = new Winch();
    private final Launcher launcher = new Launcher();
    private final Pivot pivot = new Pivot();
    private final Mailbox mailbox = new Mailbox();
    private final Collector collector = new Collector();

    private final TestAuto testAuto = new TestAuto();
    private SendableChooser<Command> autoChooser;

    private final RotateAndX rotateAndX = new RotateAndX(s_Swerve);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick atari = new Joystick(2);
    // The robot's subsystems and commands are defined here...
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    private final JoystickButton atari1 = new JoystickButton(atari, 1);
    private final JoystickButton atari2 = new JoystickButton(atari, 2);
    private final JoystickButton atari3 = new JoystickButton(atari, 3);
    private final JoystickButton atari14 = new JoystickButton(atari, 4);



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
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("testAuto", testAuto);

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // driveA.whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));
        // driveY.whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));

        // driveB.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));
        // driveX.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));

        opLeftBumper.onTrue(new InstantCommand(() -> collector.intake()));
        opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

        
       // opRightBumper.onTrue(new InstantCommand(() -> mailbox.sendToLauncher()));
       // opRightBumper.onFalse(new InstantCommand(() -> mailbox.stop()));

        // opRightBumper.onTrue(new InstantCommand(() -> collector.outtake()));
        // opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));
       // opX.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
        

        // opY.onTrue(new InstantCommand(() -> collector.pivotUp()));
        // opY.onFalse(new InstantCommand(() -> collector.pivotStop()));

        // opA.onTrue(new InstantCommand(() -> collector.pivotDown()));
        // opA.onFalse(new InstantCommand(() -> collector.pivotStop()));
        
        opY.onTrue(new InstantCommand(() -> pivot.pivotUp()));
        opY.onFalse(new InstantCommand(() -> pivot.pivotStop()));

        opA.onTrue(new InstantCommand(() -> pivot.pivotDown()));
        opA.onFalse(new InstantCommand(() -> pivot.pivotStop()));
    
        opB.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opB.onFalse(new InstantCommand(() -> launcher.stopLauncher()));

        opX.onTrue(new InstantCommand(() -> launcher.setLauncherReference()));
        opX.onFalse(new InstantCommand(() -> launcher.setLauncherReferenceToZero()));
        
        //opB.onTrue(Commands.runOnce(() -> collector.deployIntake(), collector));

        //opX.onTrue(Commands.runOnce(() -> collector.retractIntake(), collector));

        //opRightBumper.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));



       // opY.onTrue(new InstantCommand(() -> elevator.moveUp()));
        //opY.onFalse(new InstantCommand(() -> elevator.stop()));

       // opA.onTrue(new InstantCommand(() -> elevator.moveDown()));
       // opA.onFalse(new InstantCommand(() -> elevator.stop()));

      // atari1.onTrue(Commands.runOnce(() -> pivot.setPivotPosition(30), pivot));
        atari1.whileTrue( pivot.quasistaticRoutine(Direction.kForward));
       atari2.whileTrue(pivot.quasistaticRoutine(Direction.kReverse));

       atari3.whileTrue(pivot.dynamicRoutine(Direction.kForward));
       atari14.whileTrue(pivot.dynamicRoutine(Direction.kReverse));
       //atari1.onFalse(Commands.runOnce(() -> pivot., null))
    }

    public Joystick getDriveController(){
        return driver;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}