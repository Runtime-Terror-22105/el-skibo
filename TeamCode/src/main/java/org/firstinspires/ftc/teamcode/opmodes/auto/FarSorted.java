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
//
//    @Override
//    public void preInit() {
//        CommandScheduler.getInstance().schedule(new SetCameraStreamingCommand(robot.camera, false, true));
//    }

    @Override
    public boolean wantsAutoSort() {
        return false;
    }

    @Override
    public boolean wantsCamera() {
        return false;
    }

    @Override
    protected Command createAutoCommand(AutoBuildState state) {
        // TODO: REmove the following line!!!
//        robot.camera.setGlyph(CameraSubsystem.GLYPH.PGP);

        return new SequentialCommandGroup(
                new InstantCommand(() -> {robot.camera.setAprilTagsEnabled(true);
                robot.camera.setGlyphScanningEnabled(true);}),
                FarAutoBuilder.shootPreloadSortedAuto(state, ShootPathFlag.SOTM, ShootPathFlag.EARLY_LEAVE),
//                new SetCameraStreamingCommand(robot.camera, true, false),

                // we shoot at a different spot that goes better into the 2nd spike mark
                FarAutoBuilder.cycleWall(state, false, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.FORWARD_FACING_SHOOT_SPOT),
                new InstantCommand(() -> {
                    //robot.shooter.USE_SOTM = false;
                    robot.setAutoSort(true);
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
                    robot.setAutoSort(false);
                }),

                //needs to be changed to use vision in the future
                NearAutoBuilder.intakeWall(state, true),
                FarAutoBuilder.shootWall(state, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE, ShootPathFlag.FIRST_WALL_SORTED),

                FarAutoBuilder.cycleWall(state, true, ShootPathFlag.EARLY_SHOOT, ShootPathFlag.EARLY_LEAVE)


        ).alongWith(new KillTimerCommand(robot));
    }
}