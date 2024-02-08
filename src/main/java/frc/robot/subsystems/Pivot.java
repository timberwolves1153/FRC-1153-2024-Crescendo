package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Pivot extends SubsystemBase{
    
    private CANSparkMax m_leftPivot, m_rightPivot;
    private SparkPIDController pivotPID;
    private DutyCycleEncoder pivotAbsoluteEncoder;

    private PIDController pivotController;
    private ArmFeedforward pivotFF;
    private final double pivotOffset = 0.45886;
    
    private SysIdRoutine pivotRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
    );
    
    private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> mutableDistance = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> mutableVelocity = mutable(DegreesPerSecond.of(0));

    public Pivot() {
        m_leftPivot = new CANSparkMax(51, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(52, MotorType.kBrushless);

        // NEED TO TUNE
        pivotController = new PIDController(0, 0, 0);
        pivotFF = new ArmFeedforward(0, 0, 0);
        
        pivotAbsoluteEncoder = new DutyCycleEncoder(0);

        configMotors();
    }

    public void pivotUp() {
        m_leftPivot.setVoltage(-3);
    }

    public void pivotDown() {
        m_leftPivot.setVoltage(3);
    }

    public void pivotStop() {
        m_leftPivot.setVoltage(0);
        
    }

    public double getAbsoluteMeasurement() {
        return (pivotAbsoluteEncoder.getAbsolutePosition() + 0.5) % 1;
    }

    public double getPivotRadians() {
        return (getAbsoluteMeasurement() - pivotOffset) * -2 * Math.PI ;
    }

    public double getPivotDegrees() {
        return Math.toDegrees(getPivotRadians());
    }

    public double getPivotRPM_Radians() {
        return m_leftPivot.getEncoder().getVelocity() //how fast the motor is spinning in RPM
            * 2 * Math.PI / 60; //Formula for radians 
    }

    public double getPivotRPM_Degrees() {
        return m_leftPivot.getEncoder().getVelocity() //how fast the motor is spinning in RPM
            * 360 / 60; //Formula for degrees 
    }

    public void setPivotPosition(double degrees) {
        double setpointRads = Math.toRadians(degrees);
        double feedback = pivotController.calculate(getPivotRadians(), setpointRads);
        double feedforward = pivotFF.calculate(setpointRads, 1);
        m_leftPivot.setVoltage(feedback + feedforward);
    }

    @Override
    public void periodic() {
        if (Constants.launcherPivotTuningMode) {
            SmartDashboard.putNumber("pivot degrees", getPivotDegrees());
            SmartDashboard.putNumber("pivot absolute", getAbsoluteMeasurement());
            SmartDashboard.putNumber("pivot radians", getPivotRadians());
            SmartDashboard.putNumber("pivot setpoint", pivotController.getSetpoint());

        }
    }


    public void configMotors() {
        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();
        m_leftPivot.clearFaults();
        m_leftPivot.setIdleMode(IdleMode.kBrake);
        m_leftPivot.setInverted(false);

        
        m_rightPivot.clearFaults();
        m_rightPivot.setIdleMode(IdleMode.kBrake);
        m_rightPivot.follow(m_leftPivot, false);
        
        m_rightPivot.burnFlash();
        m_leftPivot.burnFlash();
    }

    /* Called by the SysIdRoutine */
    private void voltageDrive(Measure<Voltage> voltage) {
        m_leftPivot.setVoltage(voltage.in(Volts));
    }

    /* Called by the SysId routine */
    private void logMotors(SysIdRoutineLog log) {
        log.motor("pivot")
        // Log voltage
        .voltage(
            mutableAppliedVoltage.mut_replace(
                // getAppliedOutput return the duty cycle which is from [-1, +1]. We multiply this
                // by the voltage going into the spark max, called the bus voltage to receive the
                // output voltage
                m_leftPivot.getAppliedOutput() * m_leftPivot.getBusVoltage(), Volts))
        .angularPosition(mutableDistance.mut_replace(getPivotDegrees(), Degrees))
        .angularVelocity(mutableVelocity.mut_replace(getPivotRPM_Degrees(), DegreesPerSecond));
    }
    
}
