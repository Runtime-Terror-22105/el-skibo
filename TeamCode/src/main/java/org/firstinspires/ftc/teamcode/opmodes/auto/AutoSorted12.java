package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public abstract class AutoSorted12 extends OneAutoToRuleThemAll {
    public static long SHOOTING_DELAY = 2500;

    protected AutoSorted12(Team team) {
        super(team);
    }

    @Override
    public StartConfig getStartConfig() {
        return StartConfig.NEAR;
    }

    @Override
    public boolean wantsAutoSort() {
        return true;
    }

    @Override
    protected Command createAutoCommand(AutoBuilder builder) {
        builder.waitBeforeShooting(SHOOTING_DELAY);
        return new SequentialCommandGroup(
                builder.shootPreload(),
                builder.cycleSpike(1, ShootPathFlag.PRELOAD_SHOOT_SPOT),
                builder.cycleSpike(2, ShootPathFlag.PRELOAD_SHOOT_SPOT),
                // ppg is more optimal in case we miss in earlier rounds by maximizing purples
                new InstantCommand(() -> {
                    robot.camera.setGlyph(CameraSubsystem.GLYPH.PPG);
                    robot.camera.setAprilTagsEnabled(false);
                }),
                builder.cycleSpike(3, ShootPathFlag.PRELOAD_SHOOT_SPOT/*ShootPathFlag.LAST*/)
        );
    }
}