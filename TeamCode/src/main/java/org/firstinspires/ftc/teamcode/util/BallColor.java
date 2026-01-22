package org.firstinspires.ftc.teamcode.util;

public enum BallColor {
    PURPLE('P'),
    GREEN('G'),
    NONE('N');

    private final char c;

    BallColor(char c) {
        this.c = c;
    }

    public char toChar() {
        return this.c;
    }

    public static BallColor fromChar(char c) {
        for (BallColor color : BallColor.values()) {
            if (color.c == c) {
                return color;
            }
        }
        return NONE;
    }
}
