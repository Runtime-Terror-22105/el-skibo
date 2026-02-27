package org.firstinspires.ftc.teamcode.robot.auto;

public enum ShootPathFlag {
    // This is the last path in the sequence.
    LAST,

    // Next path is a horizontal intake. Only valid for near zone.
    NEXT_HORIZ,

    // Only valid for preload
    SOTM,
    
    // Be more aggressive with when we begin to shoot
    EARLY_SHOOT,

    // Be more aggressive with when we leave from the shoot position
    EARLY_LEAVE
}
