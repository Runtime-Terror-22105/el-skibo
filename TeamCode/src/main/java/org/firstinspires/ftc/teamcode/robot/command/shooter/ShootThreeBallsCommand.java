package org.firstinspires.ftc.teamcode.robot.command.shooter;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.ChangeSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerPoleActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SetSpindexerRampActive;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.WaitForSpindexerYawCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class ShootThreeBallsCommand extends SequentialCommandGroup {
    public static double spindexLoadingPower=0.5; // speed we set the SPINDEXER to spin and load into the shooter

    public SpindexerSubsystem spindexer;

    public ShooterSubsystem shooter;

    private final Robot robot;

    public static double SPINDEX_ROTATIONS = -4.5;  // revolutions, negative bc clockwise

    public ShootThreeBallsCommand(Robot robot) {
        super(
                new ChangeSpindexerYawCommand(robot.spindexer, SPINDEX_ROTATIONS*2*Math.PI),
                new ParallelRaceGroup( // keep going for either 2 rotations or until all balls are gone
                        new WaitForSpindexerYawCommand(robot.spindexer)
//                        new WaitUntilCommand(() -> {
//                            char[] balls = robot.spindexer.getBallPositions();
//                            return balls[0] == 'N' && balls[1] == 'N' && balls[2] == 'N';
//                        })
                ),

                // reset spindexer, intake, shooter, and pole
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.spindexer.setYaw(robot.spindexer.getPosition())),
                        new SetIntakeSpeedCommand(robot.intake, 0),
                        new InstantCommand(() -> {
                            robot.hardware.shooterLeft.setPower(0);
                            robot.hardware.shooterRight.setPower(0);
                        }),
                        new SetSpindexerPoleActive(robot.spindexer, false),
                        new SetSpindexerRampActive(robot.spindexer, false)
                )
        );

        this.robot=robot;
    }

    public void execute(){
        spindexer.setSpindexerPower(spindexLoadingPower);
    }
}