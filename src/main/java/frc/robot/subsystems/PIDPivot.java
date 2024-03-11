package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.lib.Interpolation.InterpolatingDouble;
import frc.robot.lib.math.LauncherInterpolation;
import frc.robot.subsystems.AprilTags.WeekZeroVision;

public class PIDPivot extends PIDSubsystem{
    
    private CANSparkMax m_leftPivot, m_rightPivot;
    private DutyCycleEncoder pivotEncoder;   
    private WeekZeroVision vision;
    private static LauncherInterpolation pivotMap;
    //private final double UNIT_CIRCLE_OFFSET = Math.toRadians(112);
    private final double FINAL_UNIT_CIRCLE_OFFSET =  Math.toRadians(80);
    private final double SUBWOOFER_DEGREES = 56;
    private final Pigeon2 encoder = new Pigeon2(7);

    public PIDPivot() {
        super(new PIDController(20, 0.01, 0.01));


        m_leftPivot = new CANSparkMax(51, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(52, MotorType.kBrushless);
        vision = new WeekZeroVision();
        pivotMap = new LauncherInterpolation();
        pivotEncoder = new DutyCycleEncoder(0);

        configMotors();
        getController().setSetpoint(Math.toRadians(getPigeonMeasurement()));
        disable();
    }
    


    public void configMotors() {
        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();
        m_leftPivot.clearFaults();
        m_leftPivot.setIdleMode(IdleMode.kBrake);
        m_leftPivot.setInverted(false);
        m_leftPivot.setSmartCurrentLimit(40);


        
        m_rightPivot.clearFaults();
        m_rightPivot.setIdleMode(IdleMode.kBrake);
        m_rightPivot.follow(m_leftPivot, false);
        m_rightPivot.setSmartCurrentLimit(40);
        m_rightPivot.burnFlash();
        m_leftPivot.burnFlash();
    }

    public void pivotUp() {
        disable();
        m_leftPivot.setVoltage(2);
    }

    public void pivotDown() {
        disable();
        // if (getDegrees() < 18.1) {
        //     m_leftPivot.setVoltage(0);
        // } else {
            m_leftPivot.setVoltage(-2);
      //  }
    }

    public void pivotStop() {
        disable();
        m_leftPivot.setVoltage(0);
        
    }

    



    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
        //PIDmovePivot(MathUtil.clamp(output, -8, 12.3));
        PIDmovePivot(MathUtil.clamp(output, -4, 4));
    }

    public void PIDmovePivot(double volts) {
        double adjustedVolts = volts;
            //negative goes up & positive goes down PLIMPTON ABS ENCODER THEORY
        double constantV;
        double clampedVolts = MathUtil.clamp(adjustedVolts, -2, 2);
        if (clampedVolts > 0) {
            constantV = 0.18;
            m_leftPivot.setVoltage(clampedVolts + constantV);
        } else if (clampedVolts < 0) {
            constantV = 0.0;
            m_leftPivot.setVoltage(clampedVolts - 0);
        }
        
    }

    public double getPitch() {
        return encoder.getPitch().getValueAsDouble();
    }
    public double getYaw() {
        return encoder.getYaw().getValueAsDouble();
    }
    public double getRoll() {
        return encoder.getRoll().getValueAsDouble();
    }




    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
       return Math.toRadians(getPigeonMeasurement());
    }

 //   public double getAbsoluteMeasurement() {
 //       return (pivotEncoder.getAbsolutePosition() + 0.6) %1;
 //   }

      public double getPigeonMeasurement() {
        return 180 - (encoder.getRoll().getValueAsDouble());
    }

 //   public double getPivotRadians() {
 //       return ((getAbsoluteMeasurement() * 2 * Math.PI)/2) - FINAL_UNIT_CIRCLE_OFFSET;
 //   }

//    public double getDegrees() {
//        return Math.toDegrees(getPivotRadians());
//    }

    public void setSetpointDegrees(double degrees) {
        double newSetpoint = Math.toRadians(degrees);
        setSetpoint(newSetpoint);
        getController().reset();
        enable();
    }

    public void interpolateSetpoint() {
       double newSetpoint =  Math.toRadians(SUBWOOFER_DEGREES);
       if (vision.isConnected()) {
           double interpolatedSetpoint = pivotMap.pivotMap.getInterpolated(new InterpolatingDouble(vision.calculateRange())).value;
           newSetpoint = Math.toRadians(interpolatedSetpoint);
       }
       setSetpoint(newSetpoint);
       getController().reset();
       enable();
    }

    public void holdPosition() {
        setSetpoint(getMeasurement());
        getController().reset();
        enable();
    }

    public void incrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.25);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(newSetpoint + oldSetpoint);
        getController().reset();
        enable();
    }

    public void decrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.25);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(oldSetpoint - newSetpoint);
        getController().reset();
        enable();
    }

    public boolean isPivotReadyToShoot() {
        if (getController().atSetpoint()) {
            return true;
        } else {
            return false;
        }
     }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Pivot Ready", isPivotReadyToShoot());
        SmartDashboard.putNumber("adjusted pigeon", getPigeonMeasurement());
        SmartDashboard.putNumber("adjusted pigeon rads", Math.toRadians(getPigeonMeasurement()));
        SmartDashboard.putNumber("Pivot PID Controller", getController().getSetpoint());
        SmartDashboard.putNumber("Pivot PID measure", getMeasurement());
        if (Constants.launcherPivotTuningMode) {
            //SmartDashboard.putNumber("pivot degrees", getDegrees());
            // SmartDashboard.putNumber("pivot absolute", getAbsoluteMeasurement());
            // SmartDashboard.putNumber("pivot radians", getPivotRadians());
            // SmartDashboard.putNumber("pivot setpoint", getController().getSetpoint());
    }

}
}
