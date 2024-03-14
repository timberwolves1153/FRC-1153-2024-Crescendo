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
   // private final ReturnFromAmp returnFromAmp = new ReturnFromAmp(pidPivot, baseClef);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick atari = new Joystick(2);
    // The robot's subsystems and commands are defined here...
    private final JoystickButton rotateWithTag = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
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
    private final JoystickButton atari14 = new JoystickButton(atari, 4);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> rotateWithTag.getAsBoolean(),
                () -> driveA.getAsBoolean(),
                () -> driveX.getAsBoolean(),
                vision
            )
        );

       

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
        NamedCommands.registerCommand("Ready Wing Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(24), pidPivot));
        NamedCommands.registerCommand("Ready Close Shot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(42), pidPivot));

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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
        driveX.onTrue(new InstantCommand(() -> winch.resetEncoder()));

        // INTAKE
        opLeftBumper.onTrue(new InstantCommand(() -> collector.intake(), collector));
        opLeftBumper.onTrue(Commands.runOnce(() -> collector.deployIntake(), collector));
        opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
        opLeftBumper.onFalse(Commands.runOnce(() -> collector.retractIntake(), collector));
        opLeftBumper.onTrue(mailboxCheck);
        opLeftBumper.onFalse(new InstantCommand(()-> mailbox.stop()));
        opLeftBumper.whileTrue(new DriverIntakeFeedback(collector, mailbox, driver, operator));

                
        opRightBumper.onTrue(new InstantCommand(() -> collector.outtake(), collector));
        opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

        opLeftStick.onTrue(new InstantCommand(() -> mailbox.sendToLauncher(), mailbox));
        opLeftStick.onTrue(new InstantCommand(() -> collector.intake(), collector));
        
        opLeftStick.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));
        opLeftStick.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

        opStart.onTrue(new InstantCommand(() -> mailbox.sendToIntake(), mailbox));
        opStart.onFalse(new InstantCommand(() -> mailbox.stop(), mailbox));

        opSelect.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
        opSelect.onTrue(new InstantCommand(() -> baseClef.resetEncoder()));
        
        //PIVOTS
        povRight.onTrue(new InstantCommand(() -> collector.pivotUp(), collector));
        povRight.onFalse(new InstantCommand(() -> collector.pivotStop(), collector));

        povLeft.onTrue(new InstantCommand(() -> collector.pivotDown(), collector));
        povLeft.onFalse(new InstantCommand(() -> collector.pivotStop(), collector));
        

    
        //LAUNCHER - when auto shoot is not working
        opX.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opX.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));
        opX.whileTrue(interpolateToSpeaker); 
        opX.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
        
                    // driveX.whileTrue(Commands.runOnce(() -> 
                    // pidPivot.setSetpointDegrees(SmartDashboard.getNumber("PIGEON MAIL",20)), pidPivot));
        // back up if interpolation is wrong/messed up

        // BASE CLEF (AMP MECH)
        opY.onTrue(new InstantCommand(() -> baseClef.manualDeploy()));
        opY.onFalse(new InstantCommand(() -> baseClef.stop()));
        opB.onTrue(new InstantCommand(() -> baseClef.manualRetract()));
        opB.onFalse(new InstantCommand(() -> baseClef.stop()));

        //launcher override
        opLeftTrigger.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opLeftTrigger.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));

        opRightTrigger.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opRightTrigger.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));
        opRightTrigger.whileTrue(autoShoot);// for some reason auto shoot wants to be called before interpolate to speaker
        opRightTrigger.whileFalse(new InstantCommand(() -> mailbox.stop()));
        opRightTrigger.whileTrue(interpolateToSpeaker); 
        opRightTrigger.whileFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));

        // mailbox pivot override
        povUp.onTrue(new InstantCommand(() -> pidPivot.pivotUp(), pidPivot));
        povUp.onFalse(new InstantCommand(() -> pidPivot.pivotStop(), pidPivot));
        //povUp.onFalse(Commands.runOnce(() -> pidPivot.holdPosition(), pidPivot));
        povDown.onTrue(new InstantCommand(() -> pidPivot.pivotDown(), pidPivot));
        povDown.onFalse(new InstantCommand(() -> pidPivot.pivotStop(), pidPivot));
        //povDown.onFalse(Commands.runOnce(() -> pidPivot.holdPosition(), pidPivot));

        //AMP
       opA.onTrue(new InstantCommand(() -> launcher.slowLaunchWithVolts()));
        opA.onFalse(new InstantCommand(() -> launcher.stopLaunchWithVolts()));
    //     //opA.whileTrue(pivotToAmp);
    //     // opA.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(42), pidPivot));
    //     // opA.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(19.5), pidPivot));
    //     // opA.whileTrue(pivotToAmp);
    //     // opA.whileFalse(returnFromAmp);
    //     // opA.onTrue(Commands.runOnce(() -> baseClef.deployClef(), baseClef));
    //     // opA.onFalse(Commands.runOnce(() -> baseClef.retractClef(), baseClef));
        opA.onTrue(
            Commands.runOnce(() -> baseClef.deployClef())
            .andThen(new WaitCommand(1))
            .andThen(Commands.runOnce(() -> pidPivot.setSetpointDegrees(42))));
        opA.onFalse(
            Commands.runOnce(() -> pidPivot.setSetpointDegrees(30))
            .andThen(new WaitCommand(0.55))
            .andThen(Commands.runOnce(() -> baseClef.stowingClef()))
            .andThen(new WaitCommand(0.5))
            .andThen(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22)))
            .andThen(new WaitCommand(0.5))
            .andThen(Commands.runOnce(() -> baseClef.retractClef())));

        // CLIMB

        // opY.whileTrue(PivotToClimb);
        // opY.whileFalse(Commands.runOnce(() -> pidPivot.holdPosition(), pidPivot));

        driveLeftTrigger.onTrue(Commands.runOnce(() -> winch.pidWinchUp(), winch));
        driveLeftTrigger.onFalse(new InstantCommand(() -> winch.stop()));

        driveRightTrigger.onTrue(Commands.runOnce(() -> winch.pidWinchDown(), winch));
        driveRightTrigger.onFalse(new InstantCommand(() -> winch.stop()));

        driveLeftBumper.onTrue(new InstantCommand(() -> winch.winchUp(), winch));
        driveLeftBumper.onFalse(new InstantCommand(() -> winch.stop(), winch));

        driveRightBumper.onTrue(new InstantCommand(() -> winch.winchDown(), winch));
        driveRightBumper.onFalse(new InstantCommand(() -> winch.stop(), winch));

        driveY.onTrue(new InstantCommand(() -> winch.rightWinchUp()));
        driveY.onFalse(new InstantCommand(() -> winch.rightStop()));
        
        driveB.onTrue(new InstantCommand(() -> winch.rightWinchDown()));
        driveB.onFalse(new InstantCommand(() -> winch.rightStop()));

        driveStart.onTrue(new InstantCommand(() -> winch.leftWinchUp()));
        driveStart.onFalse(new InstantCommand(() -> winch.leftStop()));
        
        driveSelect.onTrue(new InstantCommand(() -> winch.leftWinchDown()));
        driveSelect.onFalse(new InstantCommand(() -> winch.leftStop()));
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