package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class transfercommand extends InstantCommand {
    SpindexerSubsystem spindexer;
    public transfercommand(SpindexerSubsystem spindexer) {
        this.spindexer=spindexer;
    }

    public void phase1(){
        spindexer.setWallActive();
        spindexer.Oilup();
    }

    public void phase2(){
        spindexer.sortBalls();
    }

    public void phase3(){
        spindexer.enableRamp();
    }

    public void phase4(){
        spindexer.setYaw(0.20);
    }

    public void phase5(){
        spindexer.setSpindexerPower(spindexer.spindexTransferPower);
    }

    public void setupTransfer(){
        phase1();
        phase2();
        phase3();
        phase4();
    }



}
