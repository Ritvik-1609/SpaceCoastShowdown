package frc.robot.AutonomousRoutines;

import java.util.stream.Collector;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Lemonlight;
import frc.robot.commands.CollectCnd;
import frc.robot.commands.ShootCMD;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    ShooterSubsystem shooter;
    DriveSubsystem drive;
    CollectorSubsystem collector;
    String TwoBallAutoJSON = "2BallAuto";

    Lemonlight lemonlight = new Lemonlight();
    
    public TwoBallAuto (CollectorSubsystem collectorSubsystem, ShooterSubsystem shooter, DriveSubsystem drive){
        this.shooter = shooter;
        this.collector = collectorSubsystem;
        this.drive = drive;

        addCommands(
            new ParallelCommandGroup(
                drive.autoPath(TwoBallAutoJSON),
                new CollectCnd(.9, collectorSubsystem, false, false)
            ),
            new ShootCMD(shooter, collector, lemonlight.VtoRPM(), .9)
        );
    }
    
}
