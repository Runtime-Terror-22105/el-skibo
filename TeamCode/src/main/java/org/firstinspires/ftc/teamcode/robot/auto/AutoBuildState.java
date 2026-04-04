package org.firstinspires.ftc.teamcode.robot.auto;

import static org.firstinspires.ftc.teamcode.robot.auto.AutoConstants.SHOOT_BRAKING_STRENGTH;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.util.StartConfig;

public class AutoBuildState {
    public final Pose2d startPoseBlue;
    public final Robot robot;
    public final boolean mirror;
    public final OneAutoToRuleThemAll auto;

    public PathChain lastPath;
    public long waitBeforeShooting;
    public long prepareShootTimeBeforeReverseIntake;
    public double shootBrakingStrength;

    public AutoBuildState(OneAutoToRuleThemAll auto, Robot robot, Team team, StartConfig initial) {
        this.auto = auto;
        this.robot = robot;
        // NB: We do not mirror the start pose here because the path builder's mirror parameter handles it.
        this.startPoseBlue = initial.getStartPoseBlue();
        this.mirror = Team.RED.equals(team);
        this.lastPath = null;
        this.waitBeforeShooting = 0;
        this.prepareShootTimeBeforeReverseIntake = PrepareShootCommand.TIME_BEFORE_REVERSE_INTAKE;
        this.shootBrakingStrength = SHOOT_BRAKING_STRENGTH;
    }
}

