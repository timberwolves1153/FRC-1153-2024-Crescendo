package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Mailbox;

public class DriverIntakeFeedback extends Command{
    
    private Collector collector;
    private boolean bannerSensor;
    private Mailbox mailbox;
    private Joystick driver;
    private Joystick operator;

    public DriverIntakeFeedback (Collector collector, Mailbox mailbox, Joystick driver, Joystick operator){

        this.collector = collector;
        this.mailbox = mailbox;
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void execute() {
        if (mailbox.getBannerSensor()) {
            driver.setRumble(RumbleType.kBothRumble, 1);
            operator.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
            operator.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
    }
}
