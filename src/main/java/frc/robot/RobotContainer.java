// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Winch;
import frc.robot.subsystems.AprilTags.Vision;
import frc.robot.subsystems.AprilTags.WeekZeroVision;
import frc.robot.subsystems.AprilTags.Vision.Hardware;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.PIDPivot;
//import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.Auto.TestAuto;
import frc.robot.commands.RotateAndX;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.lib.util.AxisButton;
import frc.robot.subsystems.Collector;

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
    private final Winch elevator = new Winch();
    private final Launcher launcher = new Launcher();
    //private final Pivot pivot = new Pivot();
    private final PIDPivot pidPivot = new PIDPivot();
    private final Mailbox mailbox = new Mailbox();
    private final Collector collector = new Collector();
    private final WeekZeroVision vision = new WeekZeroVision(s_Swerve);

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
    private final JoystickButton rotateWithTag = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
     private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

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
    private final JoystickButton start = new JoystickButton(operator, XboxController.Button.kStart.value);
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
                () -> robotCentric.getAsBoolean(),
                () -> rotateWithTag.getAsBoolean(),
                vision
            )
        );
       

        NamedCommands.registerCommand("Run Launcher", new InstantCommand(() -> launcher.launchWithVolts()));
        NamedCommands.registerCommand("Subwoofer", Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
        NamedCommands.registerCommand("Index", new InstantCommand(() -> mailbox.sendToLauncher()));
        NamedCommands.registerCommand("Stop Index", new InstantCommand(() -> mailbox.stop()));
        NamedCommands.registerCommand("Deploy Intake", Commands.runOnce(() -> collector.deployIntake(), collector));
        NamedCommands.registerCommand("Run Intake", new InstantCommand(() -> collector.intake()));
        NamedCommands.registerCommand("PivotHome", Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
        NamedCommands.registerCommand("Podium", Commands.runOnce(() -> pidPivot.setSetpointDegrees(34.5), pidPivot));
        NamedCommands.registerCommand("End Launcher", new InstantCommand(() -> launcher.stopLauncher()));
        NamedCommands.registerCommand("End Mailbox", new InstantCommand(() -> mailbox.stop()));
        NamedCommands.registerCommand("Retract Intake", Commands.runOnce(() -> collector.retractIntake(), collector));
        NamedCommands.registerCommand("End Intake", new InstantCommand(() -> collector.collectorStop()));
        

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

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
        opLeftBumper.onTrue(Commands.runOnce(() -> collector.deployIntake(), collector));
        opLeftBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));
        opLeftBumper.onFalse(Commands.runOnce(() -> collector.retractIntake(), collector));
        
        opRightBumper.onTrue(new InstantCommand(() -> collector.outtake()));
        opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));

        opLeftStick.onTrue(new InstantCommand(() -> mailbox.sendToLauncher()));
        opLeftStick.onTrue(new InstantCommand(() -> collector.intake()));
        opLeftStick.onFalse(new InstantCommand(() -> mailbox.stop()));
        opLeftStick.onFalse(new InstantCommand(() -> collector.collectorStop()));

        start.onTrue(new InstantCommand(() -> mailbox.sendToIntake()));
        start.onFalse(new InstantCommand(() -> mailbox.stop()));

        // opRightBumper.onTrue(new InstantCommand(() -> collector.outtake()));
        // opRightBumper.onFalse(new InstantCommand(() -> collector.collectorStop()));
       // opX.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
        

        povRight.onTrue(new InstantCommand(() -> collector.pivotUp()));
        povRight.onFalse(new InstantCommand(() -> collector.pivotStop()));

        povLeft.onTrue(new InstantCommand(() -> collector.pivotDown()));
        povLeft.onFalse(new InstantCommand(() -> collector.pivotStop()));
        
        povUp.onTrue(new InstantCommand(() -> pidPivot.pivotUp()));
        povUp.onFalse(new InstantCommand(() -> pidPivot.pivotStop()));

        povDown.onTrue(new InstantCommand(() -> pidPivot.pivotDown()));
        povDown.onFalse(new InstantCommand(() -> pidPivot.pivotStop()));

        opX.whileTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));
        opY.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(34.5), pidPivot));
        opY.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
        opY.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opY.onFalse(new InstantCommand(() -> launcher.stopLauncher()));

        opB.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(40), pidPivot));
        opB.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
        opB.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
        opB.onFalse(new InstantCommand(() -> launcher.stopLauncher()));
        // opX.onTrue(new InstantCommand(() -> launcher.setLauncherReference()));
        // opX.onFalse(new InstantCommand(() -> launcher.setLauncherReferenceToZero()));
        
       

        //opRightBumper.onTrue(new InstantCommand(() -> collector.resetIntakeEncoder()));



       // opY.onTrue(new InstantCommand(() -> elevator.moveUp()));
        //opY.onFalse(new InstantCommand(() -> elevator.stop()));

       // opA.onTrue(new InstantCommand(() -> elevator.moveDown()));
       // opA.onFalse(new InstantCommand(() -> elevator.stop()));

    //    opY.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(26.7), pidPivot));
    //    opY.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
    //    opY.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
    //    opY.onFalse(new InstantCommand(() -> launcher.stopLauncher()));

       opA.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(56), pidPivot));
       opA.onFalse(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
       opA.onTrue(new InstantCommand(() -> launcher.launchWithVolts()));
       opA.onFalse(new InstantCommand(() -> launcher.stopLauncher()));
    //     atari1.whileTrue(pivot.quasistaticRoutine(Direction.kForward));
    //    atari2.whileTrue(pivot.quasistaticRoutine(Direction.kReverse));

    //    atari3.whileTrue(pivot.dynamicRoutine(Direction.kForward));
    //    atari14.whileTrue(pivot.dynamicRoutine(Direction.kReverse));
       //atari1.onFalse(Commands.runOnce(() -> pivot., null))
    }

    public Joystick getDriveController(){
        return driver;
      }

      public Launcher getLauncher() {
        return launcher;
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