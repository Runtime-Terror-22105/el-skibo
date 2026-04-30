package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.ANGLE_NEAR_TURRET_BLUE;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuildState;
import org.firstinspires.ftc.teamcode.robot.auto.KillTimerCommand;
import org.firstinspires.ftc.teamcode.robot.auto.NearAutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public abstract class Auto18 extends OneAutoToRuleThemAll {
    protected Auto18(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.shooter.sotmOverride = true),
                new InstantCommand(() -> robot.shooter.sotmAccelOverride = true),
                NearAutoBuilder.shootPreload(state),
                new InstantCommand(() -> {robot.shooter.isAutoTurretOn = false;}),
                new InstantCommand(() -> {robot.shooter.setTurretAngle(state.mirror ? Math.PI-ANGLE_NEAR_TURRET_BLUE : ANGLE_NEAR_TURRET_BLUE);}),
                NearAutoBuilder.cycleSpike(state, 2),
                NearAutoBuilder.cycleGate(state, true),
                NearAutoBuilder.cycleGate(state, true),
                NearAutoBuilder.cycleGate(state, true),
                new InstantCommand(() -> {robot.shooter.isAutoTurretOn = true;}),
                NearAutoBuilder.cycleSpike(state, 1, ShootPathFlag.LAST)
        ).alongWith(new KillTimerCommand(robot));
    }
}