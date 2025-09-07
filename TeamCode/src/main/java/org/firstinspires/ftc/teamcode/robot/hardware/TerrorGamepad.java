package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.firstinspires.ftc.teamcode.robot.hardware.TerrorGamepad.State.HOLDING;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.Contract;

public class TerrorGamepad {
    private final Gamepad curGamepad = new Gamepad();
    private final Gamepad lastGamepad = new Gamepad();

    public enum State {
        HOLDING,
        RISING,
        FALLING
    }

    public void update(Gamepad gamepad) {
        lastGamepad.copy(curGamepad);
        curGamepad.copy(gamepad);
    }

    @Contract(pure = true)
    public static boolean logic(boolean curState, boolean lastState, @NonNull State state) {
        switch (state) {
            case HOLDING:
                return curState;
            case RISING:
                return curState && !lastState;
            case FALLING:
                return !curState && lastState;
        }
        throw new RuntimeException("Invalid state");
    }

    public void setLedColor(int r, int g, int b, int maxValue) {
        this.curGamepad.setLedColor(r, g, b, maxValue);
    }

    public boolean a() {
        return a(HOLDING);
    }

    public boolean b() {
        return b(HOLDING);
    }

    public boolean x() {
        return x(HOLDING);
    }

    public boolean y() {
        return y(HOLDING);
    }

    public boolean dpad_left() {
        return dpad_left(HOLDING);
    }

    public boolean dpad_right() {
        return dpad_right(HOLDING);
    }

    public boolean dpad_down() {
        return dpad_down(HOLDING);
    }

    public boolean dpad_up() {
        return dpad_up(HOLDING);
    }

    public boolean atRest() {
        return atRest(HOLDING);
    }

    public boolean a(State state) {
        return !curGamepad.start && logic(curGamepad.a, lastGamepad.a, state);
    }

    public boolean x(State state) {
        return !curGamepad.start && logic(curGamepad.x, lastGamepad.x, state);
    }

    public boolean y(State state) {
        return !curGamepad.start && logic(curGamepad.y, lastGamepad.y, state);
    }

    public boolean b(State state) {
        return !curGamepad.start && logic(curGamepad.b, lastGamepad.b, state);
    }

    public boolean circle(State state) {
        return !curGamepad.start && logic(curGamepad.circle, lastGamepad.circle, state);
    }

    public boolean guide() {
        return guide(HOLDING);
    }

    public boolean guide(State state) {
        return logic(curGamepad.guide, lastGamepad.guide, state);
    }

    public boolean options() {
        return options(HOLDING);
    }

    public boolean options(State state) {
        return logic(curGamepad.options, lastGamepad.options, state);
    }

    public boolean back() {
        return back(HOLDING);
    }

    public boolean back(State state) {
        return logic(curGamepad.back, lastGamepad.back, state);
    }

    public boolean start() {
        return start(HOLDING);
    }

    public boolean start(State state) {
        return logic(curGamepad.start, lastGamepad.start, state);
    }
    public boolean dpad_left(State state) {
        return !curGamepad.start && logic(curGamepad.dpad_left, lastGamepad.dpad_left, state);
    }

    public boolean dpad_right(State state) {
        return !curGamepad.start && logic(curGamepad.dpad_right, lastGamepad.dpad_right, state);
    }

    public boolean dpad_down(State state) {
        return !curGamepad.start && logic(curGamepad.dpad_down, lastGamepad.dpad_down, state);
    }

    public boolean dpad_up(State state) {
        return !curGamepad.start && logic(curGamepad.dpad_up, lastGamepad.dpad_up, state);
    }

    public boolean atRest(State state) {
        return logic(curGamepad.atRest(), lastGamepad.atRest(), state);
    }

    public float left_trigger() {
        return curGamepad.left_trigger;
    }

    public float right_trigger() {
        return curGamepad.right_trigger;
    }

    public boolean left_trigger(float minVal, float maxVal) {
        return left_trigger(minVal, maxVal, HOLDING);
    }

    public boolean left_trigger(float minVal, float maxVal, State state) {
        boolean cur = curGamepad.left_trigger <= maxVal && curGamepad.left_trigger >= minVal;
        boolean last = lastGamepad.left_trigger <= maxVal && lastGamepad.left_trigger >= minVal;
        return logic(cur, last, state);
    }

    public boolean right_trigger(float minVal, float maxVal) {
        return right_trigger(minVal, maxVal, HOLDING);
    }

    public boolean right_trigger(float minVal, float maxVal, State state) {
        boolean cur = curGamepad.right_trigger <= maxVal && curGamepad.right_trigger >= minVal;
        boolean last = lastGamepad.right_trigger <= maxVal && lastGamepad.right_trigger >= minVal;
        return logic(cur, last, state);
    }

    public boolean left_bumper() {
        return left_bumper(HOLDING);
    }

    public boolean left_bumper(State state) {
        return logic(curGamepad.left_bumper, lastGamepad.left_bumper, state);
    }

    public boolean right_bumper() {
        return right_bumper(HOLDING);
    }

    public boolean right_bumper(State state) {
        return logic(curGamepad.right_bumper, lastGamepad.right_bumper, state);
    }

    public float left_stick_x() {
        return curGamepad.left_stick_x;
    }

    public float left_stick_y() {
        return curGamepad.left_stick_y;
    }

    public float right_stick_x() {
        return curGamepad.right_stick_x;
    }

    public float right_stick_y() {
        return curGamepad.right_stick_y;
    }

    public boolean right_stick_button() {
        return right_stick_button(HOLDING);
    }

    public boolean right_stick_button(State state) {
        return logic(curGamepad.right_stick_button, lastGamepad.right_stick_button, state);
    }

    public boolean left_stick_button() {
        return left_stick_button(HOLDING);
    }

    public boolean left_stick_button(State state) {
        return logic(curGamepad.left_stick_button, lastGamepad.left_stick_button, state);
    }

}
