package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{
    
    private CANSparkMax m_leftRoller, m_rightRoller;

    private SparkPIDController leftRollerPID;
    private PIDController rollerController;
    private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> mutableVelocity = mutable(RPM.of(0));
    private final MutableMeasure<Angle> mutablePosition = mutable(Degrees.of(0));
    private SimpleMotorFeedforward rollerFF;
    private double leftLauncherVolts, rightLauncherVolts, kp, velSetpoint;
    private SparkPIDController revController;
    
    
    // private final SysIdRoutine launcherRoutine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
    //     m_leftRoller.setVoltage(volts.in(Volts));
    //     m_rightRoller.setVoltage(volts.in(Volts));
    //  }, 
    //  log -> {
    //     log.motor("left launcher")
    //     // Log voltage
    //     .voltage(
    //         mutableAppliedVoltage.mut_replace(
    //             // getAppliedOutput return the duty cycle which is from [-1, +1]. We multiply this
    //             // by the voltage going into the spark max, called the bus voltage to receive the
    //             // output voltage
    //             m_leftRoller.getAppliedOutput(), Volts))
    //             .angularPosition(mutablePosition.mut_replace(getLeftAngularPositionDegrees(), Degrees))
    //             .angularVelocity(mutableVelocity.mut_replace(getLeftVelocity(), RPM));

    //     log.motor("right launcher")
    //     .voltage(mutableAppliedVoltage.mut_replace(
    //             m_rightRoller.getAppliedOutput(), Volts))
    //         .angularPosition(mutablePosition.mut_replace(getRightAngularPositionDegrees(), Degrees))
    //         .angularVelocity(mutableVelocity.mut_replace(getRightVelocity(), RPM));
    //   }, this)

    // );
    
    


    public Launcher() {
        m_leftRoller = new CANSparkMax(53, MotorType.kBrushless);
        m_rightRoller = new CANSparkMax(54, MotorType.kBrushless);

        // NEED TO TUNE
        rollerController = new PIDController(kp, 0, 0);
        revController = m_leftRoller.getPIDController();
        revController.setP(0.01);
        revController.setFeedbackDevice(m_leftRoller.getEncoder());
        //rollerFF = new SimpleMotorFeedforward(0, 0.2, 1.14);
        rollerFF = new SimpleMotorFeedforward(0, 0, 0);
        m_leftRoller.getEncoder().setPosition(0);
        leftLauncherVolts = 0;
        rightLauncherVolts = 0;
        velSetpoint = 0;

        // SmartDashboard.putNumber("Left Launcher Volts", leftLauncherVolts);
        // SmartDashboard.putNumber("Right Launcher Volts", rightLauncherVolts);
       // SmartDashboard.putNumber("launcher kP", kp);
       // SmartDashboard.putNumber("launcher setpoint", velSetpoint);

        configMotors();
    }

    public void launchWithVolts() {
        m_leftRoller.setVoltage(-12);
        m_rightRoller.setVoltage(-10.8);
       
    }

    public void stopLauncher() {
        m_leftRoller.setVoltage(0);
        m_rightRoller.setVoltage(0);
    }

    public double getLeftVelocity() {
        return m_leftRoller.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return m_rightRoller.getEncoder().getVelocity();
    }

    public void setLauncherVelocity() {
        
        double feedback = rollerController.calculate(getLeftVelocity(), velSetpoint);
        double feedforward = rollerFF.calculate(velSetpoint);
        
        m_leftRoller.setVoltage(feedback + feedforward);
        m_rightRoller.setVoltage(feedback + feedforward);
        
    }
    public void setLauncherZero() {
        double feedback = rollerController.calculate(getLeftVelocity(), 0);
        double feedforward = rollerFF.calculate(0);
        m_leftRoller.setVoltage(feedback + feedforward);
        m_rightRoller.setVoltage(feedback + feedforward);
        
    }

    public void setLauncherReference() {
        revController.setReference(1000, CANSparkBase.ControlType.kVelocity, 0, rollerFF.calculate(1000), SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void setLauncherReferenceToZero() {
        revController.setReference(0, CANSparkBase.ControlType.kVelocity, 0, rollerFF.calculate(0), SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void periodic() {
        if (Constants.launcherRollerTuningMode) {
            SmartDashboard.putNumber("left velocity", getLeftVelocity());
            SmartDashboard.putNumber("right Velocity", getRightVelocity());
            SmartDashboard.putNumber("current setpoint", rollerController.getSetpoint());
            SmartDashboard.putNumber("left volts", m_leftRoller.getAppliedOutput() * 12);
            SmartDashboard.putNumber("right volts", m_rightRoller.getAppliedOutput() * 12);
            
        }

    //    double leftRollerV = SmartDashboard.getNumber("Left Launcher Volts", leftLauncherVolts);
    //     leftLauncherVolts = leftRollerV;

    //     if((leftLauncherVolts != leftRollerV)) { 
    //         leftLauncherVolts = leftRollerV;
    //      }

    //      double rightRollerV = SmartDashboard.getNumber("Right Launcher Volts", rightLauncherVolts);

    //      if((rightLauncherVolts != rightRollerV)) { 
    //         rightLauncherVolts = rightRollerV;
    //      }

    //     double launcherKP = SmartDashboard.getNumber("launcher kP", kp);

    //     if (kp != launcherKP) {
    //         kp = launcherKP;
    //     }

    //     double launcherRPM = SmartDashboard.getNumber("launcher setpoint", velSetpoint);
        
    //     if (velSetpoint != launcherRPM) {
    //         velSetpoint = launcherRPM;
    //     }
    }


    public void configMotors() {
        m_leftRoller.restoreFactoryDefaults();
        m_leftRoller.clearFaults();
        m_leftRoller.setIdleMode(IdleMode.kCoast);
        m_leftRoller.setInverted(false);
        m_leftRoller.setSmartCurrentLimit(40);

        m_rightRoller.restoreFactoryDefaults();
        m_rightRoller.clearFaults();
        m_rightRoller.setIdleMode(IdleMode.kCoast);
        m_rightRoller.setInverted(true);
        m_rightRoller.setSmartCurrentLimit(40);
        // m_rightRoller.follow(m_leftRoller, true);

        // m_rightRoller.getEncoder().setPositionConversionFactor(1 / m_rightRoller.getEncoder().getCountsPerRevolution());
        // m_leftRoller.getEncoder().setPositionConversionFactor( 1 / m_leftRoller.getEncoder().getCountsPerRevolution());

        m_leftRoller.burnFlash();
        m_rightRoller.burnFlash();
    }

    /* Called by the SysIdRoutine */
    private void voltageDrive(Measure<Voltage> voltage) {
        m_leftRoller.setVoltage(voltage.in(Volts));
        m_rightRoller.setVoltage(voltage.in(Volts));
    }

    // public Command dynamicLauncher(SysIdRoutine.Direction direction) {
    //     return launcherRoutine.dynamic(direction);
    // }

    // public Command quasistaticLauncher(SysIdRoutine.Direction direction) {
    //     return launcherRoutine.quasistatic(direction);
    // }

    /* Called by the SysId routine */
    // private void logMotors(SysIdRoutineLog log) {
    //     log.motor("left launcher")
    //     // Log voltage
    //     .voltage(
    //         mutableAppliedVoltage.mut_replace(
    //             // getAppliedOutput return the duty cycle which is from [-1, +1]. We multiply this
    //             // by the voltage going into the spark max, called the bus voltage to receive the
    //             // output voltage
    //             m_leftRoller.getAppliedOutput() * m_leftRoller.getBusVoltage(), Volts))
    //     .angularVelocity(mutableVelocity.mut_replace(getLeftVelocity(), RPM));

    //     log.motor("right launcher")
    //     .voltage(mutableAppliedVoltage.mut_replace(
    //             m_rightRoller.getAppliedOutput() * m_rightRoller.getBusVoltage(), Volts))
    //         .angularVelocity(mutableVelocity.mut_replace(getRightVelocity(), RPM));
    // }
    
}
