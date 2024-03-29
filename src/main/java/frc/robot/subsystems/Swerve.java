package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;

import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.math.kinematics.SwerveModulePosition; looking into this, where are we getting "getModulePositions" from?
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private SwerveDriveKinematics kinematics;
    public Pigeon2 gyro;
    public Alliance alliance;
    public double voltage;

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> driveForVoltage(volts.in(Volts)), 
            log -> {
                log.motor("mod 0")
                .voltage(appliedVoltage.mut_replace(mSwerveMods[0].getDriveAppliedOutput(), Volts))
                .linearPosition(distance.mut_replace(mSwerveMods[0].getDriveDistance(), Meters))
                .linearVelocity(velocity.mut_replace(mSwerveMods[0].getDriveVelocity(), MetersPerSecond));

                log.motor("mod 1")
                .voltage(appliedVoltage.mut_replace(mSwerveMods[1].getDriveAppliedOutput(), Volts))
                .linearPosition(distance.mut_replace(mSwerveMods[1].getDriveDistance(), Meters))
                .linearVelocity(velocity.mut_replace(mSwerveMods[1].getDriveVelocity(), MetersPerSecond));

                log.motor("mod 2")
                .voltage(appliedVoltage.mut_replace(mSwerveMods[2].getDriveAppliedOutput(), Volts))
                .linearPosition(distance.mut_replace(mSwerveMods[2].getDriveDistance(), Meters))
                .linearVelocity(velocity.mut_replace(mSwerveMods[2].getDriveVelocity(), MetersPerSecond));

                log.motor("mod 3")
                .voltage(appliedVoltage.mut_replace(mSwerveMods[3].getDriveAppliedOutput(), Volts))
                .linearPosition(distance.mut_replace(mSwerveMods[3].getDriveDistance(), Meters))
                .linearVelocity(velocity.mut_replace(mSwerveMods[3].getDriveVelocity(), MetersPerSecond));
            }, this));
    
    public Swerve() {
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.clearStickyFaults();
        zeroGyro();

        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getModulePositions());
        // Drive base radius needs to be configured
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(10), 
                new PIDConstants(10), 
                4.5, 
                0.406, 
                new ReplanningConfig()), 
            () -> {
                var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            this);
        
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), translation.getY(), rotation, getAngle())
                                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getAngle(), getModulePositions(), pose);

        
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getAngle() {
        return (Constants.Swerve.invertGyro) ? 
        Rotation2d.fromDegrees(360 - gyro.getAngle()) : 
        Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void driveForVoltage(double volts) {
         for(SwerveModule mod : mSwerveMods) {
        //     if (mod.moduleNumber == 0 || mod.moduleNumber == 1) {
        //         mod.setDriveVoltage(volts);
        //     } else {
                mod.setDriveVoltage(volts);
            // }
            
            
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void xPosition(boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates =
            new SwerveModuleState[]{
                new SwerveModuleState(1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(45))
            };

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        System.out.println("Set to X Position");
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getAngle(), getModulePositions());  
if (Constants.swerveTuningMode) {
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " absoluteEncoderPorts", mod.getAbsoluteEncoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
           
            
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
            SmartDashboard.putNumber("test velocity" + mod.moduleNumber, mod.getDriveVelocity());
            //SmartDashboard.putNumber("Mod" + mod.moduleNumber + " get encoder 1", mod.getDriveEncoderPositon());

        }

        SmartDashboard.putNumber("Gyro Angle", getAngle().getDegrees());
    }
    }
}

//fix getModulePositions => exists within 6328, commented out function above, why no worky?