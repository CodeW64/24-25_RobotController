package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Calls the given function upon the press of the button. Most useful inside 
 * of a loop or the {@code loop()} method of an OpMode where invoking the 
 * function that is provided to the {@code ButtonPressHandler}'s constructor 
 * directly inside of that loop will create too many calls, causing problems.
 * 
 * <p>The {@code onPressFn} should be a lambda function with one paramater
 * 
 * @version 1.1.0
 */
public class ButtonPressHandler {
    private String buttonName = null;
    private Gamepad gamepad = null;
    private Consumer<Gamepad> onPress = null;

    /**
     * Is true if and only if {@code onPress} has been invoked and the button is
     * being held.
     */
    private boolean hasActivatedFunction = false;

    /**
     * Builds a new handler for the given gamepad, button name, and function to be
     * executed upon press of the button.
     * 
     * @param gamepadToUse
     * @param buttonNameToCheck
     * @param onPressFn
     * @throws NoSuchFieldException
     * @throws NullPointerException
     * @version 1.0.0
     */
    public ButtonPressHandler(
        Gamepad gamepadToUse, 
        String buttonNameToCheck, 
        Consumer<Gamepad> onPressFn
    ) throws NoSuchFieldException, NullPointerException {
        // Checking arguments for validity
        gamepadToUse.getClass().getField(buttonNameToCheck); // Ignore value, throws when not found or null
        if(onPressFn == null) {
            throw new NullPointerException("Attempted to access null \"onPressFn\" argument.");
        }

        gamepad = gamepadToUse;
        buttonName = buttonNameToCheck;
        onPress = onPressFn;
    }

    /**
     * Activates the onPress lambda only if the button has just been pressed. 
     * The button should be a boolean value; otherwise, the method will throw 
     * an exception.
     * 
     * @throws NoSuchFieldException
     * @throws IllegalAccessException
     * @version 1.0.0
     * @see activateIfPressed(Function{@literal <Field, Boolean>})
     */
    public void activateIfPressed() throws NoSuchFieldException, IllegalAccessException {
        final boolean isPressingButton = gamepad
            .getClass()
            .getField(buttonName)
            .getBoolean(gamepad);
        
        if(isPressingButton && !hasActivatedFunction) {
            hasActivatedFunction = true;
            onPress.accept(gamepad);
            return;
        }
        
        if(!isPressingButton && hasActivatedFunction) {
            hasActivatedFunction = false;
        }
    }

    /**
     * Activates the onPress lambda only if the button has just been pressed. 
     * Whether or not the button is being pressed is determined by the 
     * {@code condition} paramter.
     * 
     * @param condition Function{@literal <Field, Boolean>} - Takes the given 
     * value of the button in the gamepad object and returns whether the button is
     * being pressed or not.
     * @throws NoSuchFieldException
     * @version 1.0.0
     */
    public void activateIfPressed(Function<Field, Boolean> condition) throws NoSuchFieldException {
        final boolean isPressingButton = condition.apply(gamepad.getClass().getField(buttonName));
        if(isPressingButton && !hasActivatedFunction) {
            hasActivatedFunction = true;
            onPress.accept(gamepad);
            return;
        }

        if(!isPressingButton && hasActivatedFunction) {
            hasActivatedFunction = false;
        }
    }

    /**
     * Returns the button name given in the constructor.
     * 
     * @version 1.0.0
     */
    public String getButtonName() {
        return buttonName;
    }

    /**
     * Returns the gamepad given in the constructor.
     * 
     * @version 1.0.0
     */
    public Gamepad getGamepad() {
        return gamepad;
    }

    /**
     * Returns the consumer lambda given in the constructor.
     * 
     * @version 1.0.0
     */
    public Consumer<Gamepad> getOnPress() {
        return onPress;
    }
}