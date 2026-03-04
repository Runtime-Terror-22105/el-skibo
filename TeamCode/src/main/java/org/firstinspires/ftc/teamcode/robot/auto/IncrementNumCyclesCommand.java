package org.firstinspires.ftc.teamcode.robot.auto;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.opmodes.auto.OneAutoToRuleThemAll;

public class IncrementNumCyclesCommand extends InstantCommand {
    public IncrementNumCyclesCommand(OneAutoToRuleThemAll auto) {
        super(() -> auto.numCycles++);
    }
}
