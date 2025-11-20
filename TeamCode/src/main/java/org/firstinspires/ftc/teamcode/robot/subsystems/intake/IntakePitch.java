package org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public enum IntakePitch {
    UP(() -> IntakeSubsystem.UP_LEFT, () -> IntakeSubsystem.UP_RIGHT),
    DOWN(() -> IntakeSubsystem.DOWN_LEFT, () -> IntakeSubsystem.DOWN_RIGHT);

    public final Supplier<Double> left, right;

    IntakePitch(Supplier<Double> left, Supplier<Double> right) {
        this.left = left;
        this.right = right;
    }
}
