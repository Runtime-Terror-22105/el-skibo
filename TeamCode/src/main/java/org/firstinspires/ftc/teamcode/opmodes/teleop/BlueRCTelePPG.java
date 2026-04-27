package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@TeleOp(name = "🟦 Blue RC TeleOp PPG 🟣🟣🟢", group = "ATeleOp")
@Config
public class BlueRCTelePPG extends TerrorTeleOp {

    public BlueRCTelePPG(){
        super(Team.BLUE, CameraSubsystem.GLYPH.PPG);
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
    }
}
