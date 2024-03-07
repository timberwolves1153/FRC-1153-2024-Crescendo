package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    
    private TalonFX m_leftLauncher, m_rightLauncher;
    private VoltageOut leftOutput, rightOutput;
    private final VelocityVoltage m_leftRequest = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
    private final VelocityVoltage m_rightRequest =  new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
    private Slot0Configs slot0Configs;
    private TalonFXConfiguration leftMotorConfigs, rightMotorConfig;

    public Launcher() {

        m_leftLauncher = new TalonFX(53);
        m_rightLauncher = new TalonFX(54);

       var leftMotorConfigs = new TalonFXConfiguration();
        var rightMotorConfig = new TalonFXConfiguration();

        slot0Configs = new Slot0Configs();
        
        leftOutput = new VoltageOut(0);
        rightOutput = new VoltageOut(0);
        
        
        
        
        
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        slot0Configs.kP = 0.01;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0; //tune
        slot0Configs.kV = 0.113; //from CTRE docs + Windham

        m_leftLauncher.getConfigurator().apply(slot0Configs);
        m_leftLauncher.getConfigurator().apply(leftMotorConfigs);
        m_rightLauncher.getConfigurator().apply(slot0Configs);
        m_rightLauncher.getConfigurator().apply(rightMotorConfig);

        
    }


    public void launchWithVolts() {
        m_leftLauncher.setControl(new VoltageOut(10));
        m_rightLauncher.setControl(new VoltageOut(6.25));
    }

    public void slowLaunchWithVolts() {
        m_leftLauncher.setControl(new VoltageOut(3.5));
        m_rightLauncher.setControl(new VoltageOut(3.5));
    }

    public void stopLaunchWithVolts() {
        m_leftLauncher.setControl(leftOutput.withOutput(0));
        m_rightLauncher.setControl(rightOutput.withOutput(0));
    }

    public void closeLaunchSpeed() {
        m_leftLauncher.setControl(leftOutput.withOutput(7));
        m_rightLauncher.setControl(rightOutput.withOutput(4.375));
    }

    public void subwooferNoSpin() {
        m_leftLauncher.setControl(leftOutput.withOutput(7));
        m_rightLauncher.setControl(rightOutput.withOutput(7));
    }


    public void idleLaunchWithVolts() {
        m_leftLauncher.setControl(leftOutput.withOutput(2.5));
        m_rightLauncher.setControl(rightOutput.withOutput(2.5));
    }

    public double getLeftLauncherAppliedVolts() {
       return m_leftLauncher.getMotorVoltage().getValueAsDouble();
       
    }

    public double getRightLauncherAppliedVolts() {
       
       return m_rightLauncher.getMotorVoltage().getValueAsDouble();
    }

    public void launchWithVelocity() {
        m_leftLauncher.setControl(m_leftRequest.withVelocity(20).withFeedForward(20));
        m_rightLauncher.setControl(m_rightRequest.withVelocity(10).withFeedForward(20));
    }

    public void stop() {
        m_leftLauncher.setControl(m_leftRequest.withVelocity(0).withFeedForward(0));
        m_rightLauncher.setControl(m_rightRequest.withVelocity(0).withFeedForward(0));
    }


    public boolean isLauncherReadyToShootFar() {
        if (m_leftLauncher.getMotorVoltage().getValueAsDouble() > 9.5) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isLauncherReadyToShootClose() {
        if (m_leftLauncher.getMotorVoltage().getValueAsDouble() > 6.75) {
            return true;
        } else {
            return false;
        }
    }

   @Override
   public void periodic() {

    // double launcherV = SmartDashboard.getNumber("Launcher Volts", launcherVolts);

    //     if((launcherVolts != launcherV)) { 
    //         launcherVolts = launcherV;
    //      }

    SmartDashboard.putBoolean("Launcher Far Ready", isLauncherReadyToShootFar());
    SmartDashboard.putBoolean("Launcher Close Ready", isLauncherReadyToShootClose());
    SmartDashboard.putNumber("left roller Vel", m_leftLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right roller Vel", m_rightLauncher.getVelocity().getValueAsDouble());
   }
}
