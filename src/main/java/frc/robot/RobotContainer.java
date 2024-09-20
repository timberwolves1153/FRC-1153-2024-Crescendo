// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.subsystems.Winch;
import frc.robot.subsystems.AprilTags.Vision;
import frc.robot.subsystems.AprilTags.WeekZeroVision;
import frc.robot.subsystems.AprilTags.Vision.Hardware;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.ObjectDetecting;
import frc.robot.subsystems.PIDPivot;
//import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Winch;
import frc.robot.Auto.TestAuto;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ConstantInterpolation;
import frc.robot.commands.DriverIntakeFeedback;
import frc.robot.commands.InterpolateToSpeaker;
import frc.robot.commands.Launch;
import frc.robot.commands.MailboxCheck;
import frc.robot.commands.MailboxClimbingPosition;
import frc.robot.commands.PivotToAmp;
import frc.robot.commands.ReturnFromAmp;
import frc.robot.commands.RotateAndX;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WaitCommand;
import frc.robot.lib.math.LauncherInterpolation;
import frc.robot.lib.util.AxisButton;
import frc.robot.subsystems.BaseClef;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.KrakenPIDTest;

import java.time.Instant;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Winch winch = new Winch();
    private final Launcher launcher = new Launcher();
    //private final Pivot pivot = new Pivot();
    private final PIDPivot pidPivot = new PIDPivot();
    private final Mailbox mailbox = new Mailbox();
    private final Collector collector = new Collector();
    private final WeekZeroVision vision = new WeekZeroVision();
    private final BaseClef baseClef = new BaseClef();
   // private final ObjectDetecting objectDetecting = new ObjectDetecting();

    private final TestAuto testAuto = new TestAuto();
    private SendableChooser<Command> autoChooser;

    private final RotateAndX rotateAndX = new RotateAndX(s_Swerve);
    private final InterpolateToSpeaker interpolateToSpeaker = new InterpolateToSpeaker(pidPivot);
    private final PivotToAmp pivotToAmp = new PivotToAmp(pidPivot, baseClef);
    private final MailboxClimbingPosition PivotToClimb = new MailboxClimbingPosition(pidPivot);
    private final MailboxCheck mailboxCheck = new MailboxCheck(collector, mailbox);
    private final AutoShoot autoShoot = new AutoShoot(launcher, pidPivot, mailbox, vision);
    private final Launch launch = new Launch(launcher, vision);
    private final KrakenPIDTest subsystem = new KrakenPIDTest();
   // private final ReturnFromAmp returnFromAmp = new ReturnFromAmp(pidPivot, baseClef);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick atari = new Joystick(2);
    // The robot's subsystems and commands are defined here...
    private final JoystickButton driveRightStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveLeftStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
     private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveSelect = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final AxisButton driveLeftTrigger = new AxisButton(driver, 2, 0.5);
    private final AxisButton driveRightTrigger = new AxisButton(driver, 3, 0.5);
    private final POVButton drivePovUp = new POVButton(driver, 0);
    private final POVButton drivePovDown = new POVButton(driver, 180);
    private final POVButton drivePovRight = new POVButton(driver, 90);
    private final POVButton drivePovLeft = new POVButton(driver, 270);

    private final JoystickButton opLeftStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton opRightStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final POVButton povUp = new POVButton(operator, 0);
    private final POVButton povDown = new POVButton(operator, 180);
    private final POVButton povRight = new POVButton(operator, 90);
    private final POVButton povLeft = new POVButton(operator, 270);
    private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton opSelect = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final AxisButton opLeftTrigger = new AxisButton(operator, 2, 0.5);
    private final AxisButton opRightTrigger = new AxisButton(operator, 3, 0.5);

    private final JoystickButton atari1 = new JoystickButton(atari, 1);
    private final JoystickButton atari2 = new JoystickButton(atari, 2);
    private final JoystickButton atari3 = new JoystickButton(atari, 3);
    private final JoystickButton atari4 = new JoystickButton(atari, 4);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (!Constants.HERSH_MODE) {
        s_Swerve.setDefaultCommand(
            
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> driveRightStick.getAsBoolean(),
                () -> driveX.getAsBoolean(),
                () -> driveLeftTrigger.getAsBoolean(),
                vision));
            } else {
                s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false, 
                () -> driveLeftStick.getAsBoolean(), 
                () -> driveA.getAsBoolean(), 
                () -> driveLeftTrigger.getAsBoolean(), 
                vision));
            }
        

       

        NamedCommands.registerCommand("Run Launcher", new InstantCommand(() -> launcher.launchWithVolts()));
        NamedCommands.registerCommand("Subwoofer No Spin", new InstantCommand(() -> launcher.subwooferNoSpin()));
        NamedCommands.registerCommand("Index", new InstantCommand(() -> mailbox.sendToLauncher()));
        NamedCommands.registerCommand("Stop Index", new InstantCommand(() -> mailbox.stop()));
        NamedCommands.registerCommand("Deploy Intake", Commands.runOnce(() -> collector.deployIntake(), collector));
        NamedCommands.registerCommand("Run Intake", new InstantCommand(() -> collector.intake()));
        NamedCommands.registerCommand("PivotHome", Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
        NamedCommands.registerCommand("Subwoofer", Commands.runOnce(() -> pidPivot.setSetpointDegrees(57), pidPivot));
        NamedCommands.registerCommand("End Launcher", new InstantCommand(() -> launcher.stopLaunchWithVolts()));
        NamedCommands.registerCommand("End Mailbox", new InstantCommand(() -> mailbox.stop()));
        NamedCommands.registerCommand("Retract Intake", Commands.runOnce(() -> collector.retractIntake(), collector));
        NamedCommands.registerCommand("End Intake", new InstantCommand(() -> collector.collectorStop()));
        NamedCommands.registerCommand("Pivot Mailbox", new InstantCommand(() -> pidPivot.interpolateSetpoint()));
        NamedCommands.registerCommand("Close Launcher", new InstantCommand(() -> launcher.closeLaunchSpeed()));
        NamedCommands.registerCommand("Ready Wing Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.0), pidPivot));
        NamedCommands.registerCommand("Ready Close Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(34.5), pidPivot));
        NamedCommands.registerCommand("Ready Source Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(31), pidPivot));
        NamedCommands.registerCommand("SkipNSprint Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(22.0), pidPivot));
        NamedCommands.registerCommand("SkipNSprint Shot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(22.0), pidPivot));
        NamedCommands.registerCommand("Shoot At Wing", new InstantCommand(() -> launcher.launchAtWing()));
        

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);
        SmartDashboard.putNumber("PIGEON MAIL", pidPivot.getPigeonMeasurement());

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

        if (!Constants.HERSH_MODE) {
            //resets
                driveLeftStick.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
                opSelect.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
                opSelect.onTrue(new InstantCommand(() -> baseClef.resetEncoder()));
            // Intake
                opLeftBumper.onTrue(new InstantCommand(() -> collector.intake(), collector));
                opLeftBumper.onTrue(Commands.runOnce(() -> collector.deployIntake(), collector));
                opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
                opLeftBumper.onFalse(Commands.runOnce(() -> collector.retractIntake(), collector));
                opLeftBumper.onTrue(mailboxCheck);

                opLeftBumper.onFalse(new InstantCommand(()-> mailbox.stop()));
                opLeftBumper.whileTrue(new DriverIntakeFeedback(collector, mailbox, driver, operator));

                opRightBumper.onTrue(new InstantCommand(() -> collector.outtake(), collector));
                opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
                opRightBumper.onTrue(new InstantCommand(() -> mailbox.sendToIntake(), mailbox));
                opRightBumper.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
                
                //passing notes

                driveLeftTrigger.onTrue(new InstantCommand(() -> launcher.passNote(6.75, 4.21)));
                driveLeftTrigger.onFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts()));
                driveLeftTrigger.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
                driveLeftTrigger.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));

                // manual pivot
                povUp.onTrue(new InstantCommand(() -> pidPivot.pivotUp(), pidPivot));
                povUp.onFalse(new InstantCommand(() -> pidPivot.holdPosition(), pidPivot));
                povDown.onTrue(new InstantCommand(() -> pidPivot.pivotDown(), pidPivot));
                povDown.onFalse(new InstantCommand(() -> pidPivot.holdPosition(), pidPivot));

                povRight.onTrue(new InstantCommand(() -> collector.pivotUp(), collector));
                povRight.onFalse(new InstantCommand(() -> collector.pivotStop(), collector));
                povLeft.onTrue(new InstantCommand(() -> collector.pivotDown(), collector));
                povLeft.onFalse(new InstantCommand(() -> collector.pivotStop(), collector));
                
                // AMP

                opA.onTrue(new InstantCommand(() -> baseClef.deployClef()));
                opA.onFalse(new InstantCommand(() -> baseClef.retractClef()));
                opA.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(45), pidPivot));
                opA.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                opA.onTrue(new InstantCommand(() -> launcher.slowLaunchWithVolts()));
                opA.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));


                opLeftStick.onTrue(new InstantCommand(() -> mailbox.sendToLauncher(), mailbox));
                opLeftStick.onTrue(new InstantCommand(() -> collector.intake(), collector));
                
                opLeftStick.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
                opLeftStick.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

                opStart.onTrue(new InstantCommand(() -> mailbox.sendToIntake(), mailbox));
                opStart.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
        
        

    
                //LAUNCHER - when auto shoot is not working
                driveX.whileTrue(launch);
                driveX.whileFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts(), launcher));
                driveX.whileTrue(interpolateToSpeaker); 
                driveX.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                driveX.whileFalse(new InstantCommand(() -> mailbox.stop()));
                
                            // driveX.whileTrue(Commands.runOnce(() -> 
                            // pidPivot.setSetpointDegrees(SmartDashboard.getNumber("PIGEON MAIL",20)), pidPivot));
                // back up if interpolation is wrong/messed up

                // BASE CLEF (AMP MECH)
                opY.onTrue(new InstantCommand(() -> baseClef.manualDeploy()));
                opY.onFalse(new InstantCommand(() -> baseClef.stop()));
                opB.onTrue(new InstantCommand(() -> baseClef.manualRetract()));
                opB.onFalse(new InstantCommand(() -> baseClef.stop()));

                //launcher override
                opLeftTrigger.onTrue(new InstantCommand(() -> launcher.passNote(7, 7)));
                opLeftTrigger.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
                opLeftTrigger.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                opLeftTrigger.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));
                

                opRightTrigger.whileTrue(launch);
                opRightTrigger.whileFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts()));
                opRightTrigger.whileTrue(autoShoot);// for some reason auto shoot wants to be called before interpolate to speaker
                opRightTrigger.whileFalse(new InstantCommand(() -> mailbox.stop()));
                opRightTrigger.whileTrue(interpolateToSpeaker); 
                opRightTrigger.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));

            // CLIMBING BUTTONS
                driveLeftBumper.onTrue(new InstantCommand(() -> winch.winchUp(), winch));
                driveLeftBumper.onTrue(Commands.runOnce(() -> pidPivot.pivotStop()));
                driveLeftBumper.onFalse(new InstantCommand(() -> winch.stop(), winch));

                driveRightBumper.onTrue(new InstantCommand(() -> winch.winchDown(), winch));
                driveRightBumper.onTrue(Commands.runOnce(() -> pidPivot.pivotStop()));
                driveRightBumper.onFalse(new InstantCommand(() -> winch.stop(), winch));

                driveY.onTrue(new InstantCommand(() -> winch.rightWinchUp()));
                driveY.onFalse(new InstantCommand(() -> winch.rightStop()));
                
                driveB.onTrue(new InstantCommand(() -> winch.rightWinchDown()));
                driveB.onFalse(new InstantCommand(() -> winch.rightStop()));

                driveStart.onTrue(new InstantCommand(() -> winch.leftWinchUp()));
                driveStart.onFalse(new InstantCommand(() -> winch.leftStop()));
                
                driveSelect.onTrue(new InstantCommand(() -> winch.leftWinchDown()));
                driveSelect.onFalse(new InstantCommand(() -> winch.leftStop()));
                
        } else {
            // HERSH'S SINGLE DRIVER CONTROLS

            // Resets
                driveSelect.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
                driveSelect.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
                driveSelect.onTrue(new InstantCommand(() -> baseClef.resetEncoder()));

            // Shooting notes
                driveRightTrigger.whileTrue(launch);
                driveRightTrigger.whileFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts()));
                driveRightTrigger.whileTrue(autoShoot);// for some reason auto shoot wants to be called before interpolate to speaker
                driveRightTrigger.whileFalse(new InstantCommand(() -> mailbox.stop()));
                driveRightTrigger.whileTrue(interpolateToSpeaker); 
                driveRightTrigger.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));


            //when auto shoot is not working
                opX.whileTrue(launch);
                opX.whileFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts(), launcher));
                opX.whileTrue(interpolateToSpeaker); 
                opX.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                opX.whileFalse(new InstantCommand(() -> mailbox.stop()));
        

            // Passing Notes
                driveLeftTrigger.onTrue(new InstantCommand(() -> launcher.passNote(6.75, 4.21)));
                driveLeftTrigger.onFalse(new InstantCommand(() -> launcher.idleLaunchWithVolts()));
                driveLeftTrigger.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
                driveLeftTrigger.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));

            // send notes to launcher 

                driveLeftStick.onTrue(new InstantCommand(() -> mailbox.sendToLauncher(), mailbox));
                driveLeftStick.onTrue(new InstantCommand(() -> collector.intake(), collector));
                
                driveLeftStick.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
                driveLeftStick.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));


                //launcher override
                driveY.onTrue(new InstantCommand(() -> launcher.passNote(7, 7)));
                driveY.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
                driveY.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                driveY.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));

                // Intake
                driveRightBumper.onTrue(new InstantCommand(() -> collector.intake(), collector));
                driveRightBumper.onTrue(Commands.runOnce(() -> collector.deployIntake(), collector));
                driveRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
                driveRightBumper.onFalse(Commands.runOnce(() -> collector.retractIntake(), collector));
                driveRightBumper.onTrue(mailboxCheck);
                driveRightBumper.onFalse(new InstantCommand(()-> mailbox.stop()));
                driveRightBumper.whileTrue(new DriverIntakeFeedback(collector, mailbox, driver, operator));

                driveLeftBumper.onTrue(new InstantCommand(() -> collector.outtake(), collector));
                driveLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
                driveLeftBumper.onTrue(new InstantCommand(() -> mailbox.sendToIntake(), mailbox));
                driveLeftBumper.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
                driveLeftBumper.onTrue(new InstantCommand(() -> mailbox.sendToIntake(), mailbox));
                driveLeftBumper.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));

            // AMP

                driveA.onTrue(new InstantCommand(() -> baseClef.deployClef()));
                driveA.onFalse(new InstantCommand(() -> baseClef.retractClef()));
                driveA.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(45), pidPivot));
                driveA.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
                driveA.onTrue(new InstantCommand(() -> launcher.slowLaunchWithVolts()));
                driveA.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));

                // CLIMBING BUTTONS
                // drivePovUp.onTrue(new InstantCommand(() -> winch.winchUp(), winch));
                // drivePovUp.onTrue(Commands.runOnce(() -> pidPivot.pivotStop()));
                // drivePovUp.onFalse(new InstantCommand(() -> winch.stop(), winch));

                // drivePovDown.onTrue(new InstantCommand(() -> winch.winchDown(), winch));
                // drivePovDown.onTrue(Commands.runOnce(() -> pidPivot.pivotStop()));
                // drivePovDown.onFalse(new InstantCommand(() -> winch.stop(), winch));
        }   
    
        

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