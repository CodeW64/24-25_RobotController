package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;
import org.firstinspires.ftc.teamcode.teamprograms.teleop.IntoTheDeepTeleop;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.locks.Condition;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.function.DoubleConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Takes a preloaded sample and pushes it into the net zone. Grabs more neutral
 * samples afterwards.
 * 
 * @author Connor Larson
 */
@Autonomous(name="Basket Placer (Preloaded Specimen)")
public class SpecimenPreloadBasket extends AutoCommonPaths {
    private boolean isBlue = true;
    private boolean shouldParkObservation = true; // False: Go to ascent zone there
    private int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    private int coloredTagId = AprilLocater.COLORED_BLUE_ID;
    private boolean repositionEnabled = false;

    private ButtonPressHandler toggleBlueSide;
    private ButtonPressHandler toggleObservationPark;
    private ButtonPressHandler repositionToggle;

    private DcMotorEx linearSlideLift;

    /**
     * Describes the robot's relative coordinates. That is, the x is width on 
     * the global field plane, y is the height, and heading is the CCW rotation 
     * from the x-axis.
     */
    private final Pose2d ROBOT_INIT_POSE = new Pose2d(17, 16.5, Math.toRadians(90));
    
    /**
     * Describes the the center of the robot's relative coordinates. See 
     * ROBOT_INIT_POSE for an explanatiom
     */
    private final Pose2d ROBOT_CENTER = new Pose2d(
        ROBOT_INIT_POSE.position.times(0.5),
        ROBOT_INIT_POSE.heading
    );

    /**
     * Global coordinates of where and how the robot starts for this opmode.
     */
    private final Pose2d START_LOCATION = new Pose2d(
        -72 + ROBOT_CENTER.position.x, // -72 is the left grid x-coord 
         48 - ROBOT_CENTER.position.y, //  48 is the top y-coord
         0  + ROBOT_CENTER.heading.toDouble() // 0 is the deafult rotation
    );

    private double sampleSensingDistance;
    private int TWELVE_INCHES_EXTENSION = 1250;
    private int FULLY_RETRACTED = 0;


    /**
     * Contains methods so that the arm can be managed from outside the teleop, 
     * asynchronously.
     */
    private class LiftHandlerThread extends Thread {
        private boolean isOpen = true;
        private Boolean isWaiting = false;
        private long pressDuration = 30; // In milliseonds
        private ArrayList<Closeable> runningThreads = new ArrayList<Closeable>();

        public LiftHandlerThread() {
            super();
            setDaemon(true);
        }

        @Override
        public void run() {
            runOverridenOpMode();
        }

        public boolean hasSample() {
            return isPossessingSample(sampleSensingDistance);
        }

        public void simulateInput(Gamepad buttonPresses) throws InterruptedException {
            final Gamepad previousGamepad = new Gamepad();
            previousGamepad.copy(gamepad2);
            gamepad2 = buttonPresses;
            sleep(pressDuration);
            gamepad2 = previousGamepad;
        } 

        public void simulateInput(Gamepad buttonPresses1, Gamepad buttonPresses2) 
            throws InterruptedException 
        {
            final Gamepad previousGamepad1 = new Gamepad();
            final Gamepad previousGamepad2 = new Gamepad();
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
            gamepad1 = buttonPresses1;
            gamepad2 = buttonPresses2;
            sleep(pressDuration);
            gamepad1 = previousGamepad1;
            gamepad2 = previousGamepad2;
        }

        /**
         * Extends the slides at the given power until the position is reached.
         * 
         * @param target Where to extend/retract to.
         * @param power How powerful the motor should be run. Positive values 
         *     extend, negative retract.
         */
        public void extendSlides(int target, int tolerance, double power) {
            setIsWaiting(true);
            final ConditionalThread conditionalThread = new ConditionalThread();
            conditionalThread.finishInitialization(
                () -> Math.abs(linearSlideLift.getCurrentPosition() - target) < tolerance,
                (Boolean unusedParam) -> gamepad2.right_stick_y = -((float) power),
                (Boolean unusedParam) -> {
                    gamepad2.right_stick_y = 0;
                    setIsWaiting(false);
                    runningThreads.remove(conditionalThread);
                }
            );
            runningThreads.add(conditionalThread);
            conditionalThread.start();
        }
        
        // /**
        //  * Simulates using the left stick to pivot the slides. 
        //  * 
        //  * @param left_stick_y A value in the range [-1.0, 1.0]. Positive 
        //  *     values extend; negative retract.
        //  */
        // public void pivotSlides(double left_stick_y) {
        //     fakeGamepad.left_stick_y = -((float) left_stick_y);
        //     simulateInput(fakeGamepad);
        //     fakeGamepad.left_stick_y = 0;
        // }

        public void setIntakeToHighSpeed() throws InterruptedException {
            gamepad2.right_bumper = true;
            sleep(pressDuration);
            gamepad2.right_bumper = false;
        } 

        public void setIntakeToRestSpeed() throws InterruptedException {
            gamepad2.left_bumper = true;
            sleep(pressDuration);
            gamepad2.left_bumper = false;
        }

        public void reverseIntakeDirection() throws InterruptedException {
            gamepad2.b = true;
            sleep(pressDuration);
            gamepad2.b = false;
        }

        public void switchToDepositMode() throws InterruptedException {
            gamepad2.left_trigger = 1.0f;
            sleep(pressDuration);
            gamepad2.left_trigger = 0;
        }

        public void switchToIntakeMode() throws InterruptedException {
            gamepad2.right_trigger = 1.0f;
            sleep(pressDuration);
            gamepad2.right_trigger = 0;
        }

        public void overrideLimitSwitch() throws InterruptedException {
            gamepad2.x = true;
            gamepad2.y = true;
            sleep(pressDuration);
            gamepad2.x = false;
            gamepad2.y = false;
        }

        public void enterManualOverride() throws InterruptedException {
            gamepad2.dpad_down = true;
            gamepad2.a = true;
            sleep(pressDuration);
            gamepad2.dpad_down = false;
            gamepad2.a = false;
        }

        public void enterHangTime() throws InterruptedException {

        }
        
        public void close() {
            isOpen = false;
            for(Closeable closeableThread : runningThreads) {
                try {
                     closeableThread.close();
                } catch(IOException err) {
                    telemetry.addData("...wat o_O ", err);
                    telemetry.update();
                }
                runningThreads.remove(closeableThread);
            }
        }

        /**
         * Makes the calling thread wait for the previous call of a method on 
         * this object to have finished waiting asynchronously.
         */
        public void waitForFinish() {
            while(getIsWaiting() && isOpen) {
                // Still just waiting
            }
        }
    
        public void setPressDuration(long ms) {
            pressDuration = ms;
        }

        public synchronized void setIsWaiting(boolean isWaiting) {
            this.isWaiting = isWaiting;
        }

        public synchronized boolean getIsWaiting() {
            return isWaiting;
        }
    }

    /**
     * Waits in a sepate thread until a condition is satifsied. Once the condition is true,
     */
    private class ConditionalThread extends Thread implements Closeable {
        private BooleanSupplier condition;
        private Consumer<Boolean> onContinue = (Boolean unusedParam) -> {/* NOOP */};
        private Consumer<Boolean> onFinish;
        private boolean isOpen = true;

        public ConditionalThread() {
            super();
            setDaemon(true);
        }

        public ConditionalThread(BooleanSupplier condition, Consumer<Boolean> onFinish) {
            super();
            this.condition = condition;
            this.onFinish = onFinish;
            setDaemon(true);
        }
        
        public ConditionalThread(
            BooleanSupplier condition, 
            Consumer<Boolean> onContinue, 
            Consumer<Boolean> onFinish
        ) {
            super();
            this.condition = condition;
            this.onContinue = onContinue;
            this.onFinish = onFinish;
            setDaemon(true);
        }

        /**
         * Sets the attributes of the thread to the given arguments. This exists 
         * to suppress any "Vairable may not have been initialized" compilation 
         * errors.  
         * 
         * @param condition Ends the thread and calls finisher once true.
         * @param onContinue Is called everytime the loop iterates.
         * @param onFinish Called once the loop is finished
         */
        public void finishInitialization(
            BooleanSupplier condition, 
            Consumer<Boolean> onContinue, 
            Consumer<Boolean> onFinish
        ) {
            this.condition = condition;
            this.onContinue = onContinue;
            this.onFinish = onFinish;
        }

        @Override
        public void run() {
            boolean currentBoolean;
            while(currentBoolean = (!condition.getAsBoolean() && isOpen)) {
                onContinue.accept(currentBoolean);
            }

            onFinish.accept(!condition.getAsBoolean());
        }
    
        @Override
        public void close() {
            isOpen = false;
        }
    }

    private final LiftHandlerThread lift = new LiftHandlerThread(); 

    @Override
    public void opMode_init() {
        super.opMode_init();
        
        // Initializing other hardware(-ish) bits
        globalDrive = new MecanumDrive(hardwareMap, START_LOCATION);
        linearSlideLift = hardwareMap.get(DcMotorEx.class, "linearSlideLift");
        sampleSensingDistance = hardwareMap
            .get(ColorRangeSensor.class, "sampleSensor")
            .getDistance(DistanceUnit.CM);

        // Creating init_loop options
        try {
            toggleBlueSide = new ButtonPressHandler(gamepad1, "a", (Gamepad g) -> {
                isBlue = !isBlue;
                neutralTagId = isBlue ? AprilLocater.NEUTRAL_BLUE_ID : AprilLocater.NEUTRAL_RED_ID;
                coloredTagId = isBlue ? AprilLocater.COLORED_BLUE_ID : AprilLocater.COLORED_RED_ID;
            });

            toggleObservationPark = new ButtonPressHandler(gamepad1, "b", (Gamepad g) -> {
                shouldParkObservation = !shouldParkObservation;
            });
            
            repositionToggle = new ButtonPressHandler(gamepad1, "start", (Gamepad g) -> {
                repositionEnabled = !repositionEnabled;
            });
        } catch(NoSuchFieldException | NullPointerException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }
    }

    @Override 
    public void opMode_init_loop() {
        telemetry.addLine("==== POSITIONING NOTES ====");
        telemetry.addLine(
            "The robot should be positioned on the close edge of the tile" +
            "closest to the net zone (baskets area) without being inside it. The" + 
            "robot should be against the wall with the front facing the net zone. "
        );
        telemetry.addLine("Thank you!!! ❤️🦾");

        telemetry.addLine("");
        telemetry.addLine("==== OPTION CURRENT VALUES ====");
        telemetry.addData("Is Blue Side", isBlue);
        telemetry.addData("Should Park Observation", shouldParkObservation);
        telemetry.addData("Reposition Enabled", repositionEnabled);

        telemetry.addLine("");
        telemetry.addLine("==== OPTION CONTROLS ====");
        telemetry.addData("Toggle Blue Side", "Press " + toggleBlueSide.getButtonName());
        telemetry.addData("Toggle Observation Park", "Press " + toggleObservationPark.getButtonName());
        telemetry.addData("Respoition Toggle", "Press " + repositionToggle.getButtonName());
        telemetry.addData("[[DEV]] Await Continue", "Press start");

        // Detecting buttonPresses
        try {
            toggleBlueSide.activateIfPressed();
            toggleObservationPark.activateIfPressed();
            repositionToggle.activateIfPressed();
        } catch(NoSuchFieldException | IllegalAccessException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }

        if(repositionEnabled) {
            driveWheels();
        }
    }

    private void extendAsync() throws InterruptedException {
        lift.extendSlides(TWELVE_INCHES_EXTENSION, 20, 0.15);
        // life.waitForFinish();
    }

    private void grabSampleAsync() throws InterruptedException {
        lift.switchToIntakeMode();
    }

    private void toggleArmToDespitAsync() throws InterruptedException {
        lift.switchToDepositMode();
    }

    private void depositSync() throws InterruptedException {
        lift.switchToIntakeMode();
        lift.setIntakeToHighSpeed();
        sleep(500);
        lift.setIntakeToRestSpeed();
    }

    private void grabSample() throws InterruptedException {
        // TODO: Make this retry if nothing is grabbed
        grabSampleAsync();
        sleep(1000);
    }

    private void berriddenOfSample() throws InterruptedException {
        // The arm is raised, so put it into the basket!
        // toggleArmToDespitAsync();
        depositSync();
        sleep(500); // To stop from accidentally moving too fast
        toggleArmToDespitAsync(); // Changing arm position
    }

    private void raiseArmAsync() throws InterruptedException {
        toggleArmToDespitAsync();
    }

    /**
     * Executes the main code for the robot. This exists so the code isn't 
     * polluted with try-catches, but can be declared entirely. If an exception 
     * is thrown, it would only be reasonable in the case of op mode end, where 
     * we want all function to end anyways. 
     * 
     * @throws InterruptedException
     */
    private void main() throws InterruptedException {
        // Completely disabling the idea of hitting buttons
        toggleBlueSide = null;
        toggleObservationPark = null;
        repositionToggle = null;

        // Starting the actual stuffs
        lift.start();

        // Driving to the net zone
        // final AprilTagDetection netZoneInitial = getDetection(this.neutralTagId);
        globalDrive.updatePoseEstimate();
        moveRobotToNetZone(isBlue);
        berriddenOfSample();

        // Driving to the spike marks
        for(int i = 2; i >= 0; i--) {
            // Initial positioning data
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final double extraRotation = i == 0 ? Math.toRadians(20) : 0; // Rotate more cuz' last one's hard to get to. 
            final Pose2d intakeOffset = new Pose2d(-6.25, -1, extraRotation - Math.PI / 2); // Offset from bot center
            final Pose2d grabbingDistance = new Pose2d(-12, 0, 0);
            
            // Finding the offset
            final Pose2d totalOffset = addPoses(intakeOffset, grabbingDistance);
            final Vector2d[] rotationMatrix = {
                new Vector2d(Math.cos(extraRotation), Math.sin(extraRotation)),
                new Vector2d(-Math.sin(extraRotation), Math.cos(extraRotation))
            };
            final Vector2d rotatedPosition = transformVector(totalOffset.position, rotationMatrix);
            final Pose2d finalOffset = new Pose2d(rotatedPosition, totalOffset.heading); 
            
            telemetry.addData("Final Offset", logPose(finalOffset));

            // Moving to grab the sample
            setDestinationOffset(finalOffset);
            globalDrive.updatePoseEstimate();
            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            // Grabbing the pixel
            resetDestinationOffset();
            grabSample();
            raiseArmAsync();
            // TODO: GRab the pixel!
            
            // Scoring!
            globalDrive.updatePoseEstimate();
            moveRobotToNetZone(isBlue);
            berriddenOfSample();
        }

        // Parking
        telemetry.addData("Status", "Completed!");
        telemetry.update();

        final AprilTagDetection observationZone = getDetection(this.coloredTagId);
        if(this.shouldParkObservation) {
            globalDrive.updatePoseEstimate();
            moveRobotToObservation(observationZone, false); 
        } else {
            globalDrive.updatePoseEstimate();
            moveRobotToAscentZone(observationZone);
            globalDrive.updatePoseEstimate();
            attemptAscent(1);
        }

        // Yipee! Finishing up
        telemetry.addData("Status", "Completed! 🥳");
        telemetry.update();
    }

    @Override
    public void opMode_start() {
        try {
            main();
        } catch(InterruptedException err) {
            telemetry.addData("!! CAUGHT FATAL ERROR", err);
            telemetry.update();
        }
    }

    @Override
    public void opMode_stop() {
        powerDriveMotors(0, 0, 0, 0);
        lift.close();
    }
}
