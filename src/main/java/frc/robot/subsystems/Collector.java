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
    private CANSparkMax collectorMotor1;
    private CANSparkMax collectorMotor2;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pidController;


    public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, kInput, maxRPM;
    private final double IntakeSetpoint = 30;

    public Collector() {
        
        pivotMotor = new CANSparkMax(41, MotorType.kBrushless);
        collectorMotor1 = new CANSparkMax(40, MotorType.kBrushless);
        collectorMotor2 = new CANSparkMax(42, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pidController = pivotMotor.getPIDController();
        pivotEncoder.setPosition(0);
        pivotMotor.restoreFactoryDefaults();

        pidController.setP(0.03);
        pidController.setI(kI);
        pidController.setD(0.001);
        pidController.setFF(0);
       
        pivotMotor.setInverted(false);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.burnFlash();

        collectorMotor1.restoreFactoryDefaults();
        collectorMotor1.setInverted(false);
        collectorMotor1.setIdleMode(IdleMode.kCoast);
        collectorMotor1.setSmartCurrentLimit(40);
        collectorMotor1.burnFlash();

        collectorMotor2.restoreFactoryDefaults();
        collectorMotor2.setInverted(false);
        collectorMotor2.setIdleMode(IdleMode.kCoast);
        collectorMotor2.setSmartCurrentLimit(40);
        collectorMotor2.burnFlash();

       

        kP = 0.01;
        kI = 0;
        kD = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        

        
    }

    public void intake(){
        collectorMotor1.setVoltage(-12);
        collectorMotor2.setVoltage(12);
    }

    public void outtake(){
        collectorMotor1.setVoltage(12);
        collectorMotor2.setVoltage(-12);
    }

    public void collectorStop(){
        collectorMotor1.setVoltage(0);
        collectorMotor2.setVoltage(0);
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

    public void resetIntakeEncoder() {
        pivotEncoder.setPosition(0);
    }

    public double getIntakePosition() {
        return pivotEncoder.getPosition();
    }


    @Override
    public void periodic() {

        if (Constants.collectorTuningMode) {
        SmartDashboard.putNumber("Intake Encoder", getIntakePosition());

        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("Feed Forward", kFF);
     
        }

    }

}
