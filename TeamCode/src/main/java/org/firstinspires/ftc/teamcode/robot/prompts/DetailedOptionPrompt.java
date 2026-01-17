package org.firstinspires.ftc.teamcode.robot.prompts;

import com.skeletonarmy.marrow.internal.Button;
import com.skeletonarmy.marrow.prompts.Prompt;

public class DetailedOptionPrompt<T> extends Prompt<T> {
    private final String header;
    private final String preLine;
    private final String postLine;
    private final T[] options;
    private int selectedOptionIndex = 0;

    @SafeVarargs
    public DetailedOptionPrompt(String header, T... options) {
        this.header = header;
        this.options = options;
        this.preLine = "";
        this.postLine = "";
    }

    @SafeVarargs
    public DetailedOptionPrompt(String header, String preLine, String postLine, T... options) {
        this.header = header;
        this.options = options;
        this.preLine = preLine;
        this.postLine = postLine;
    }

    @Override
    public T process() {
        addLine(header);
        addLine(preLine);
        addLine("");

        for (int i = 0; i < options.length; i++) {
            if (i == selectedOptionIndex) {
                addLine((i + 1) + ") " + options[i] + " <");
            } else {
                addLine((i + 1) + ") " + options[i]);
            }
        }

        addLine(postLine);

        if (justPressed(Button.DPAD_UP)) {
            selectedOptionIndex = (selectedOptionIndex - 1 + options.length) % options.length;
        } else if (justPressed(Button.DPAD_DOWN)) {
            selectedOptionIndex = (selectedOptionIndex + 1) % options.length;
        }

        if (justPressed(Button.A)) {
            return options[selectedOptionIndex];
        }

        return null;
    }
}
