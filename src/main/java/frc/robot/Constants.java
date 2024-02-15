package frc.robot;


//import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.NEOSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.AprilTags.AprilTagCamera.Resolution;

public final class Constants {

    public static final boolean launcherRollerTuningMode = false;
    public static final boolean launcherPivotTuningMode = false;
    public static final boolean collectorTuningMode = false;
    public static final double stickDeadband = 0.2;

    public static final class Field {
        public static final double FIELD_WIDTH = 8.21;
        public static final double FIELD_LENGTH = 16.54;

    public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(15.64, 5.55);

    }

     public static final class VisionSettings {
        public static final String INTAKECAM_NAME = "intakeCam";
        public static final Transform3d INTAKECAM_LOCATION = new Transform3d(
            new Translation3d(0.33, 0.33, 0.2), 
            new Rotation3d(0, Math.toRadians(21.7), Math.toRadians(23)));


        public static final Resolution INTAKECAM_RESOLUTION = Resolution.RES_1280_720;
        public static final Rotation2d INTAKECAM_FOV = Rotation2d.fromDegrees(75);
    }


    public static final class Swerve {
        public static final int pigeonID = 5;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final NEOSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            NEOSwerveConstants.SDSMK4i(NEOSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelDiameter = chosenModule.wheelDiameter;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double voltageComp = 12.0;


        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean absoluteEncoderPortsInvert = chosenModule.absoluteEncoderPortsInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 30;
        public static final int anglePeakCurrentLimit = 20;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 50;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        // public static final double angleKP = chosenModule.angleKP;
        // public static final double angleKI = chosenModule.angleKI;
        // public static final double angleKD = chosenModule.angleKD;
        // public static final double angleKFF = chosenModule.angleKFF;

        public static final double angleKP = 0.06;
        public static final double angleKI = 0.00;
        public static final double angleKD = 0.02;
        public static final double angleKFF = 0.00;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.3; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;


        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.0321); //TODO: This must be tuned to specific robot
        public static final double driveKV = (0.1807);
        public static final double driveKA = (0.0615);

        public static final double driveConversionPositionFactor = 
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        /** Meters per Second */
        
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 11.5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
       // public static final double angleConversionFactor = 0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 01;
            public static final int angleMotorID = 02;
            public static final int absoluteEncoderPorts = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.77);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int absoluteEncoderPorts = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(32.9);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int absoluteEncoderPorts = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(4.77);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int absoluteEncoderPorts = 3; //MIGHT SWAP WITH 1
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(215.38);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, absoluteEncoderPorts, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
  }