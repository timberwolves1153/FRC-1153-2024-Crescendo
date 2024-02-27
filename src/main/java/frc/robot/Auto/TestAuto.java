package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;

public class TestAuto extends SequentialCommandGroup{

    private Collector collector;
    private PathPlannerPath path1;
    public TestAuto() {
        //NamedCommands.registerCommand("intake", new InstantCommand(() -> collector.intake()));

       // path1 = PathPlannerPath.fromPathFile("Three Note 1");
        

        addCommands(
           // new InstantCommand(() -> collector.intake()), AutoBuilder.followPath(path1), new InstantCommand(() -> collector.collectorStop())
           );
    }
}
