package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

public class ButtonHelper {
    private Gamepad gamepad;
    private Map<String, Boolean> previousButtonStates;

    public ButtonHelper(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.previousButtonStates = new HashMap<>();
        initializeButtonStates();
    }

    private void initializeButtonStates() {
        previousButtonStates.put("a", false);
        previousButtonStates.put("b", false);
        previousButtonStates.put("x", false);
        previousButtonStates.put("y", false);
        previousButtonStates.put("dpad_up", false);
        previousButtonStates.put("dpad_down", false);
        previousButtonStates.put("dpad_left", false);
        previousButtonStates.put("dpad_right", false);
        previousButtonStates.put("left_bumper", false);
        previousButtonStates.put("right_bumper", false);
        previousButtonStates.put("left_stick_button", false);
        previousButtonStates.put("right_stick_button", false);
        previousButtonStates.put("start", false);
        previousButtonStates.put("back", false);
        previousButtonStates.put("guide", false);
    }

    public void update() {
        previousButtonStates.put("a", gamepad.a);
        previousButtonStates.put("b", gamepad.b);
        previousButtonStates.put("x", gamepad.x);
        previousButtonStates.put("y", gamepad.y);
        previousButtonStates.put("dpad_up", gamepad.dpad_up);
        previousButtonStates.put("dpad_down", gamepad.dpad_down);
        previousButtonStates.put("dpad_left", gamepad.dpad_left);
        previousButtonStates.put("dpad_right", gamepad.dpad_right);
        previousButtonStates.put("left_bumper", gamepad.left_bumper);
        previousButtonStates.put("right_bumper", gamepad.right_bumper);
        previousButtonStates.put("left_stick_button", gamepad.left_stick_button);
        previousButtonStates.put("right_stick_button", gamepad.right_stick_button);
        previousButtonStates.put("start", gamepad.start);
        previousButtonStates.put("back", gamepad.back);
        previousButtonStates.put("guide", gamepad.guide);
    }

    public boolean isButtonJustPressed(String button) {
        Boolean previousState = previousButtonStates.get(button);
        if (previousState == null) {
            throw new IllegalArgumentException("Invalid button: " + button);
        }

        boolean currentState = getCurrentButtonState(button);
        return currentState && !previousState;
    }

    private boolean getCurrentButtonState(String button) {
        switch (button) {
            case "a":
                return gamepad.a;
            case "b":
                return gamepad.b;
            case "x":
                return gamepad.x;
            case "y":
                return gamepad.y;
            case "dpad_up":
                return gamepad.dpad_up;
            case "dpad_down":
                return gamepad.dpad_down;
            case "dpad_left":
                return gamepad.dpad_left;
            case "dpad_right":
                return gamepad.dpad_right;
            case "left_bumper":
                return gamepad.left_bumper;
            case "right_bumper":
                return gamepad.right_bumper;
            case "left_stick_button":
                return gamepad.left_stick_button;
            case "right_stick_button":
                return gamepad.right_stick_button;
            case "start":
                return gamepad.start;
            case "back":
                return gamepad.back;
            case "guide":
                return gamepad.guide;
            default:
                throw new IllegalArgumentException("Invalid button: " + button);
        }
    }
}
