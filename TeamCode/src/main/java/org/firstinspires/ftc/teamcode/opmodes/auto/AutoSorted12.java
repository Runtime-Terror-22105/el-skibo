package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.auto.ShootPathFlag;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public abstract class AutoSorted12 extends OneAutoToRuleThemAll {
    public static long SHOOTING_DELAY = 250;

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
        builder.shootBrakingStrength = AutoConstants.SORTED_BRAKING_STRENGTH;
        return new SequentialCommandGroup(
                builder.shootPreload(),
                builder.cycleSpike(1),
                builder.cycleSpike(2),
                // ppg is more optimal in case we miss in earlier rounds by maximizing purples
                new InstantCommand(() -> {
                    robot.camera.setGlyph(CameraSubsystem.GLYPH.PPG);
                    robot.camera.setAprilTagsEnabled(false);
                    robot.spindexer.overrideMaxPower = true;
                }),
                builder.cycleSpike(3, ShootPathFlag.LAST)
        );
    }
}