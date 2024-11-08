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
 * @author Connor Larson
 * @version 1.2.0
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

    private Field buttonField;

    /**
     * Builds a new handler for the given gamepad, button name, and function to be
     * executed upon press of the button.
     * 
     * @param gamepadToUse
     * @param buttonNameToCheck
     * @param onPressFn
     * @throws NoSuchFieldException The provided buttonName was not found in the gamepad class
     */
    public ButtonPressHandler(
        Gamepad gamepadToUse, 
        String buttonNameToCheck, 
        Consumer<Gamepad> onPressFn
    ) throws NoSuchFieldException {
        // Checking arguments for validity
        buttonField = gamepadToUse.getClass().getField(buttonNameToCheck); // Throws when not found or null
        gamepad = gamepadToUse;
        buttonName = buttonNameToCheck;
        onPress = onPressFn;
    }
   
    /**
     * Builds a new handler for the given gamepad, button name, and function to be
     * executed upon press of the button.
     * 
     * @param gamepadToUse
     * @param buttonNameToCheck
     * @param onPressFn
     * @throws NoSuchFieldException The provided buttonName was not found in the gamepad class
     */
    public ButtonPressHandler(
        Gamepad gamepadToUse, 
        String buttonNameToCheck
    ) throws NoSuchFieldException {
        // Checking arguments for validity
        buttonField = gamepadToUse.getClass().getField(buttonNameToCheck); // Throws when not found or null
        gamepad = gamepadToUse;
        buttonName = buttonNameToCheck;
    }

    /**
     * Returns whether the onpress should be called. The return value of the 
     * condition should be true if the button is currently being held and false 
     * otherwise. Whether it has just been pressed is determined by this object.
     * 
     * @param condition Function{@literal <Field, Boolean>} - Takes the given 
     * value of the button in the gamepad object and returns whether the button is
     * being pressed or not.
     * @return Boolean indicating whether, relative to last check, the button
     *     has just beem pressed.
     * @see isPressed()
     */
    public boolean isPressed(Function<Field, Boolean> condition) {
        final boolean isButtonHeld = condition.apply(buttonField);
        
        // The button went from not held to actively held; it was pressed!
        if(isButtonHeld && !hasActivatedFunction) {
            hasActivatedFunction = true;
            return true;
        }

        // The button is has been let go; set hasActivated to false
        if(!hasActivatedFunction) {
            hasActivatedFunction = false;
        }

        return false;
    }

    /**
     * Returns whether the onpress should be called. This relies on the button 
     * being a boolean value. If not, this will throw.
     * 
     * @return Boolean indicating whether, relative to last check, the button 
     *     has just been pressed. 
     * @throws IllegalAccessException
     * @see isPressed(Function{@literal <Field, Boolean>})
     */
    public boolean isPressed() throws IllegalAccessException {
        final boolean isButtonHeld = buttonField.getBoolean(gamepad);
        
        // The button went from not held to actively held; it was pressed!
        if(isButtonHeld && !hasActivatedFunction) {
            hasActivatedFunction = true;
            return true;
        }

        // The button is has been let go; set hasActivated to false
        if(!hasActivatedFunction) {
            hasActivatedFunction = false;
        }

        return false;
    }

    /**
     * Activates the onPress lambda only if the button has just been pressed. 
     * The button should be a boolean value; otherwise, the method will throw 
     * an exception.
     * 
     * @throws IllegalAccessException
     * @see activateIfPressed(Function{@literal <Field, Boolean>})
     */
    public void activateIfPressed() throws IllegalAccessException {
        if(isPressed() && onPress != null) {
            onPress.accept(gamepad);
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
     */
    public void activateIfPressed(Function<Field, Boolean> condition) {
        if(isPressed(condition) && onPress != null) {
            onPress.accept(gamepad);
        }
    }

    /**
     * Returns the button name given in the constructor.
     * 
     */
    public String getButtonName() {
        return buttonName;
    }

    /**
     * Returns the gamepad given in the constructor.
     * 
     */
    public Gamepad getGamepad() {
        return gamepad;
    }

    /**
     * Returns the consumer lambda given in the constructor.
     * 
     */
    public Consumer<Gamepad> getOnPress() {
        return onPress;
    }
}