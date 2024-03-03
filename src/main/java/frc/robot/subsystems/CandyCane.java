package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class CandyCane extends PIDSubsystem{
    
    private CANSparkMax candyCane;
    private DutyCycleEncoder abosluteEncoder;


    public CandyCane() {
        super(new PIDController(0, 0, 0));
        candyCane = new CANSparkMax(56, MotorType.kBrushed);
        abosluteEncoder = new DutyCycleEncoder(2);

        config();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        moveCandyCane(MathUtil.clamp(output, -6, 6));
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return abosluteEncoder.getAbsolutePosition();
    }

    public void moveCandyCane(double volts) {
        double adjustedVolts = volts;
            //negative goes up & positive goes down
        double constantV;
        double clampedVolts = MathUtil.clamp(adjustedVolts, -12, 12);
        if (clampedVolts > 0) {
            constantV = 0.15;
            candyCane.setVoltage(clampedVolts + constantV);
        } else if (clampedVolts < 0) {
            constantV = 0.0;
            candyCane.setVoltage(clampedVolts - constantV);
        }
        
    }

    public void config() {
        candyCane.restoreFactoryDefaults();
        candyCane.clearFaults();
        candyCane.setIdleMode(IdleMode.kBrake);
        candyCane.setSmartCurrentLimit(30);
        candyCane.burnFlash();
    }
}
