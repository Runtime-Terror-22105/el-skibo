package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_BRAKING_STRENGTH;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.StartConfig;

@Config
public class AutoBuilder {
    public static boolean TWO_SEGMENT_PARK_SORTED = false;

    public final Pose2d startPoseBlue;
    public final Robot robot;
    public final boolean mirror;
    public final OneAutoToRuleThemAll auto;

    public long prepareShootTimeBeforeReverseIntake = PrepareShootCommand.TIME_BEFORE_REVERSE_INTAKE;
    public double shootBrakingStrength = SHOOT_BRAKING_STRENGTH;

    private final AutoBuildState state;

    public AutoBuilder(OneAutoToRuleThemAll auto, Robot robot, Team team, StartConfig initial) {
        this.state = new AutoBuildState(auto, robot, team, initial);
        this.auto = this.state.auto;
        this.robot = this.state.robot;
        this.startPoseBlue = this.state.startPoseBlue;
        this.mirror = this.state.mirror;
    }

    private void syncTuningToState() {
        state.prepareShootTimeBeforeReverseIntake = prepareShootTimeBeforeReverseIntake;
        state.shootBrakingStrength = shootBrakingStrength;
    }

    public void waitBeforeShooting(long time) {
        state.waitBeforeShooting += time;
    }

    public PathChain getLastPath() {
        return state.lastPath;
    }

    public PathChain customPath(PathChain path) {
        return state.lastPath = path;
    }

    public Command shootPreload(ShootPathFlag... flagArr) {
        syncTuningToState();
        return NearAutoBuilder.shootPreload(state, flagArr);
    }

    public Command shootPreloadFar(ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.shootPreloadFar(state, flagArr);
    }

    public Command intakeSpike(int spikeNumber) {
        syncTuningToState();
        return NearAutoBuilder.intakeSpike(state, spikeNumber);
    }

    public Command intakeSpikeHorizontal(int spikeNumber) {
        syncTuningToState();
        return NearAutoBuilder.intakeSpikeHorizontal(state, spikeNumber);
    }

    public Command shootSpike(int spikeNumber, ShootPathFlag... flagArr) {
        syncTuningToState();
        return NearAutoBuilder.shootSpike(state, spikeNumber, flagArr);
    }

    public Command cycleSpike(int spikeNumber, ShootPathFlag... flags) {
        syncTuningToState();
        return NearAutoBuilder.cycleSpike(state, spikeNumber, flags);
    }

    public Command parkSorted() {
        syncTuningToState();
        return NearAutoBuilder.parkSorted(state);
    }

    public Command pushGate() {
        syncTuningToState();
        return NearAutoBuilder.pushGate(state);
    }

    public Command intakeGate() {
        syncTuningToState();
        return NearAutoBuilder.intakeGate(state);
    }

    public Command shootGate(boolean reverseIntake, ShootPathFlag... flagArr) {
        syncTuningToState();
        return NearAutoBuilder.shootGate(state, reverseIntake, flagArr);
    }

    public Command cycleGate(boolean reverseIntake, ShootPathFlag... flags) {
        syncTuningToState();
        return NearAutoBuilder.cycleGate(state, reverseIntake, flags);
    }

    public Command intakeSpike3Far() {
        syncTuningToState();
        return FarAutoBuilder.intakeSpike3Far(state);
    }

    public Command shootSpike3Far(ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.shootSpike3Far(state, flagArr);
    }

    public Command intakeWall(boolean reverseIntake) {
        syncTuningToState();
        return FarAutoBuilder.intakeWall(state, reverseIntake);
    }

    public Command prepareVision() {
        syncTuningToState();
        return FarAutoBuilder.prepareVision(state);
    }

    public Command intakeVision(boolean reverseIntake) {
        syncTuningToState();
        return FarAutoBuilder.intakeVision(state, reverseIntake);
    }

    public Command intakeTunnel(boolean reverseIntake) {
        syncTuningToState();
        return FarAutoBuilder.intakeTunnel(state, reverseIntake);
    }

    public Command shootWall(ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.shootWall(state, flagArr);
    }

    public Command cycleTunnel(boolean reverseIntake, ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.cycleTunnel(state, reverseIntake, flagArr);
    }

    public Command cycleWall(boolean reverseIntake, ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.cycleWall(state, reverseIntake, flagArr);
    }

    public Command cycleVision(boolean reverseIntake, ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.cycleVision(state, reverseIntake, flagArr);
    }

    public Command intakeWallLong() {
        syncTuningToState();
        return FarAutoBuilder.intakeWallLong(state);
    }

    public Command controlPathLong() {
        syncTuningToState();
        return FarAutoBuilder.controlPathLong(state);
    }

    public Command intakeTunnelLong() {
        syncTuningToState();
        return FarAutoBuilder.intakeTunnelLong(state);
    }

    public Command shootWallLong(ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.shootWallLong(state, flagArr);
    }

    public Command cycleLong(boolean reverseIntake, ShootPathFlag... flagArr) {
        syncTuningToState();
        return FarAutoBuilder.cycleLong(state, reverseIntake, flagArr);
    }
}
