package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.FarAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.NearAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.robot.auto.SortedAutoBuilder;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class FarSorted extends OneAutoToRuleThemAll {
    protected FarSorted(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.FAR_SORTED;
    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        return new SequentialCommandGroup(
                FarAutoBuilder.shootPreload(state, ShootPathFlag.SOTM, ShootPathFlag.EARLY_LEAVE),

                FarAutoBuilder.cycleWall(state, false, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE),
                new InstantCommand(() -> {
                    //robot.shooter.USE_SOTM = false;
                    //robot.shooter.USE_SOTM_ACCEL = false;
                    //robot.setAutoSort(true);
                }),
                FarAutoBuilder.intakeSpike2AndPushGate(state),
                SortedAutoBuilder.shootSpike(state, 2),

                NearAutoBuilder.intakeSpike(state, 3),
                SortedAutoBuilder.shootSpike(state, 3),

                NearAutoBuilder.intakeSpike(state, 1, ShootPathFlag.SORTING),
                SortedAutoBuilder.shootSpike(state, 1),

                new InstantCommand(() -> {
                    //robot.shooter.USE_SOTM = true;
                    //robot.shooter.USE_SOTM_ACCEL = true;
                    //robot.setAutoSort(false);
                }),

                //needs to be changed to use vision in the future
                NearAutoBuilder.intakeWall(state, true),
                FarAutoBuilder.shootWall(state, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.FIRST_WALL_SORTED),

                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)


        ).alongWith(new KillTimerCommand(robot));
    }
}