package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoFar;

@Config
@Autonomous(name="Blue Far", group="Auto Far")
public class BlueAutoFar extends AutoFar {
    public BlueAutoFar() {
        super(Team.BLUE);
    }
}
