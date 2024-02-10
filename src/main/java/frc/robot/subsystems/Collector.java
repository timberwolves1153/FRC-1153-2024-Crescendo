package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        collectorMotor = new CANSparkMax(40, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pidController = pivotMotor.getPIDController();

        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setInverted(false);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.burnFlash();

        collectorMotor.restoreFactoryDefaults();
        collectorMotor.setInverted(false);
        collectorMotor.setIdleMode(IdleMode.kCoast);
        collectorMotor.setSmartCurrentLimit(40);
        collectorMotor.burnFlash();

       

        kP = 0.1;
        kI = 0;
        kD = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        
    }

    public void intake(){
        collectorMotor.setVoltage(-12);
    }

    public void outtake(){
        collectorMotor.setVoltage(12);
    }

    public void collectorStop(){
        collectorMotor.setVoltage(0);
    }

    public void pivotUp(){
        pivotMotor.setVoltage(-1);
    }

    public void pivotDown(){
        pivotMotor.setVoltage(1);
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

        if (Constants.collectorTuningMode) {
        SmartDashboard.putNumber("Intake Encoder", pivotEncoder.getPosition());

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("Feed Forward", kFF);
     
        }

    }

}
