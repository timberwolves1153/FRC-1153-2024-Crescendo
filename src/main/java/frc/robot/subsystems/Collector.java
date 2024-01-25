package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase{
    
    private CANSparkMax pivotMotor;
    private CANSparkMax collectorMotor;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pidController;


    public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, kInput, maxRPM;
    private final double IntakeSetpoint = 0;

    public Collector() {
        
        pivotMotor = new CANSparkMax(41, MotorType.kBrushless);
        collectorMotor = new CANSparkMax(42, MotorType.kBrushless);

        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setInverted(false);
        pivotMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.burnFlash();

        collectorMotor.restoreFactoryDefaults();
        collectorMotor.setInverted(false);
        collectorMotor.setIdleMode(IdleMode.kCoast);
        collectorMotor.burnFlash();
        collectorMotor.setSmartCurrentLimit(40);

        pivotEncoder = pivotMotor.getEncoder();

        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        pidController.setP(kD);
        pidController.setI(kD);
        pidController.setD(kD);
        pidController.setFF(kD);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Minimum Output", kMinOutput);
        SmartDashboard.putNumber("Maximum Output", kMaxOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

    }

    public void intake(){
        collectorMotor.setVoltage(1);
    }

    public void outtake(){
        collectorMotor.setVoltage(-1);
    }

    public void collectorStop(){
        collectorMotor.setVoltage(0);
    }

    public void pivotUp(){
        pivotMotor.setVoltage(1);
    }

    public void pivotDown(){
        pivotMotor.setVoltage(-1);
    }

    public void pivotStop(){
        pivotMotor.setVoltage(0);
    }

    public void deployIntake() { //takes in certain number of encoder ticks
        pidController.setReference(IntakeSetpoint, ControlType.kPosition);
    }

    public void retractIntake() {
        pidController.setReference(IntakeSetpoint - IntakeSetpoint, ControlType.kPosition);
    }

    public void receiveValues() {

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double min = SmartDashboard.getNumber("MinOutput", 0);
        double max = SmartDashboard.getNumber("MaxOutput", 0);

        if (p != kP) {
            pidController.setP(kP = p);
        }

        if (i != kI) {
            pidController.setI(kI = i);
        }

        if (d != kD) {
            pidController.setD(kD = d);
        }

        if (ff != kFF) {
            pidController.setFF(kFF = ff);
        }

        if (min != kMinOutput) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        if  (max != kMaxOutput) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Encoder", pivotEncoder.getPosition());
    }

}
