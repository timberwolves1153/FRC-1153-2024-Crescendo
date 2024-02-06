package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private PIDController leftRollerController;
    
    private SimpleMotorFeedforward rollerFF;
    private double leftLauncherVolts, rightLauncherVolts, kp, velSetpoint;
    
    private SysIdRoutine launcherRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
    );
    
    private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> mutableVelocity = mutable(RPM.of(0));


    public Launcher() {
        m_leftRoller = new CANSparkMax(53, MotorType.kBrushless);
        m_rightRoller = new CANSparkMax(54, MotorType.kBrushless);

        // NEED TO TUNE
        leftRollerController = new PIDController(kp, 0, 0);
        rollerFF = new SimpleMotorFeedforward(0, 0, 0);

        leftLauncherVolts = 0;
        rightLauncherVolts = 0;
        kp = 0;
        velSetpoint = 0;

        SmartDashboard.putNumber("Left Launcher Volts", leftLauncherVolts);
        SmartDashboard.putNumber("Right Launcher Volts", rightLauncherVolts);
        SmartDashboard.putNumber("kP", kp);
        SmartDashboard.putNumber("velocity setpoint RPM", kp);

        configMotors();
    }

    public void launchWithVolts() {
        m_leftRoller.setVoltage(leftLauncherVolts);
        m_rightRoller.setVoltage(-1 * rightLauncherVolts);
       
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

    public void setLauncherVelocity(double setpoint) {
        double feedback = leftRollerController.calculate(getLeftVelocity(), setpoint);
        double feedforward = rollerFF.calculate(setpoint);
        m_leftRoller.setVoltage(feedback + feedforward);
        
    }

    @Override
    public void periodic() {
        if (Constants.launcherRollerTuningMode) {
            SmartDashboard.putNumber("roller velocity", getLeftVelocity());
            SmartDashboard.putNumber("velocity setpoint", leftRollerController.getSetpoint());
        }

       double leftRollerV = SmartDashboard.getNumber("Left Launcher Volts", leftLauncherVolts);
        leftLauncherVolts = leftRollerV;

        if((leftLauncherVolts != leftRollerV)) { 
            leftLauncherVolts = leftRollerV;
         }

         double rightRollerV = SmartDashboard.getNumber("Right Launcher Volts", rightLauncherVolts);

         if((rightLauncherVolts != rightRollerV)) { 
            rightLauncherVolts = rightRollerV;
         }
        
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

        m_leftRoller.burnFlash();
        m_rightRoller.burnFlash();
    }

    /* Called by the SysIdRoutine */
    private void voltageDrive(Measure<Voltage> voltage) {
        m_leftRoller.setVoltage(voltage.in(Volts));
        m_rightRoller.setVoltage(voltage.in(Volts));
    }

    public Command dynamicLauncher(SysIdRoutine.Direction direction) {
        return launcherRoutine.dynamic(direction);
    }

    public Command quasistaticLauncher(SysIdRoutine.Direction direction) {
        return launcherRoutine.quasistatic(direction);
    }

    /* Called by the SysId routine */
    private void logMotors(SysIdRoutineLog log) {
        log.motor("left launcher")
        // Log voltage
        .voltage(
            mutableAppliedVoltage.mut_replace(
                // getAppliedOutput return the duty cycle which is from [-1, +1]. We multiply this
                // by the voltage going into the spark max, called the bus voltage to receive the
                // output voltage
                m_leftRoller.getAppliedOutput() * m_leftRoller.getBusVoltage(), Volts))
        .angularVelocity(mutableVelocity.mut_replace(getLeftVelocity(), RPM));

        log.motor("right launcher")
        .voltage(mutableAppliedVoltage.mut_replace(
                m_rightRoller.getAppliedOutput() * m_rightRoller.getBusVoltage(), Volts))
            .angularVelocity(mutableVelocity.mut_replace(getRightVelocity(), RPM));
    }

    
}
