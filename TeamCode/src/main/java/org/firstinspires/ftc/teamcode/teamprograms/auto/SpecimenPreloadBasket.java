package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;
import org.firstinspires.ftc.teamcode.teamprograms.teleop.IntoTheDeepTeleop;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.locks.Condition;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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
    private final Pose2d ROBOT_INIT_POSE = new Pose2d(17, 16.5, Math.toRadians(0));
    
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
         0 - ROBOT_CENTER.position.y, //  48 is the top y-coord
         0  + ROBOT_CENTER.heading.toDouble() // 0 is the deafult rotation
    );

    private double sampleSensingDistance;

    private int CHAMBER_EXTENSION = (int) SLIDE_CONSTANTS.bottomBucketHeightAlternate; // TODO: Make this appropriate to the chambers
    private int TWELVE_INCHES_EXTENSION = 1150;
    private int FULLY_RETRACTED = 600;
    
    private double EXTENSION_POWER = 0.4; // Previously 0.15
    private double RETRACTION_POWER = -1.0; // Previous -0.4

    /**
     * Contains methods so that the arm can be managed from outside the teleop, 
     * asynchronously.
     */
    private class LiftHandlerThread extends CloseableThread {
        private boolean isOpen = true;
        private Boolean isExtending = false;
        private Boolean isSwitching = false;
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

        /**
         * Extends the slides at the given power until the position is reached.
         * 
         * @param target Where to extend/retract to.
         * @param tolerance The maximum allowed differnece between the target 
         *     and the motor's end position. The difference is absolute, so the 
         *     size of the allowed range is equal to 2 * tolerance. 
         * @param power How powerful the motor should be run. Positive values 
         *     extend, negative retract.
         */
        public void extendSlides(int target, int tolerance, double power) {
            setIsExtending(true);
            final ConditionalThread conditionalThread = new ConditionalThread();
            conditionalThread.finishInitialization(
                () -> Math.abs(linearSlideLift.getCurrentPosition() - target) < tolerance,
                (Boolean unusedParam) -> gamepad2.right_stick_y = -((float) power),
                (Boolean unusedParam) -> {
                    gamepad2.right_stick_y = 0;
                    setIsExtending(false);
                    runningThreads.remove(conditionalThread);
                }
            );
            runningThreads.add(conditionalThread);
            conditionalThread.start();
        }

        /**
         * Returns whether the given state of the lift is a despoit-position 
         * state. This includes exteneded, extended alternate, and retracting.
         * 
         * @param state The state to check the position of.
         * @return Boolean describing wether it is accurate to call the state a 
         *     deposit state.
         */
        public boolean isDepositPosition(AutoArmRunner.LinearSlideStates state) {
            return state == AutoArmRunner.LinearSlideStates.DEPOSIT_ACTIVE
                || state == AutoArmRunner.LinearSlideStates.DEPOSIT_RETRACT_SET
                || state == AutoArmRunner.LinearSlideStates.DEPOSIT_RETRACT
                || state == AutoArmRunner.LinearSlideStates.DEPOSIT_ALTERNATE_ACTIVE
                || state == AutoArmRunner.LinearSlideStates.DEPOSIT_ALTERNATE_RETRACT_SET;
        }
        
        /**
         * Returns the state that the arm would pivot to. The argument is true 
         * if the arm is in deposit mode and false otherwise (i.e. intake).
         * 
         * @param isDepositState True is the arm is in a deposit state
         * @return The states that the arm can pivot to (assuming no cancelation).
         */
        public AutoArmRunner.LinearSlideStates[] getTargetState(boolean isDepositState) {
            final AutoArmRunner.LinearSlideStates[] states = new AutoArmRunner.LinearSlideStates[2];
            if(isDepositState) {
                states[0] = AutoArmRunner.LinearSlideStates.INTAKE_ACTIVE;
                states[1] = AutoArmRunner.LinearSlideStates.INTAKE_FULL;
            } else {
                states[0] = AutoArmRunner.LinearSlideStates.DEPOSIT_ACTIVE;
                states[1] = AutoArmRunner.LinearSlideStates.DEPOSIT_ALTERNATE_ACTIVE;
            }
            return states;
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchArmMode() {
            // Set initial conditions
            final AutoArmRunner.LinearSlideStates intialState = linearSlideState; 
            final boolean initialStateIsDeposit = isDepositPosition(linearSlideState);
            final AutoArmRunner.LinearSlideStates[] targets = getTargetState(initialStateIsDeposit);
            setIsSwitching(true);
            
            // Initialize the process for switiching
            final ConditionalThread releaseButtonThread = new ConditionalThread();
            releaseButtonThread.finishInitialization(
                // Holds the button until the state changes
                () -> isDepositPosition(linearSlideState) != initialStateIsDeposit, 
                (Boolean unusedParam) -> gamepad2.left_trigger = 1.0f, 
                (Boolean unusedParam) -> {
                    gamepad2.left_trigger = 0;
                    runningThreads.remove(releaseButtonThread);
                }
            );
                
            final ConditionalThread endSwitchThread = new ConditionalThread();
            endSwitchThread.finishInitialization(
                // Alerts the lift that the switching is done
                () -> !linearSlideState.equals(targets[0]) && !linearSlideState.equals(targets[1]), 
                (Boolean unusedParam) -> {
                    setIsSwitching(false);
                    runningThreads.remove(endSwitchThread);
                }
            );
                
            // Starting the threads
            runningThreads.add(releaseButtonThread);
            runningThreads.add(endSwitchThread);
            releaseButtonThread.start();
            endSwitchThread.start();
        }

        @Override
        public void close() {
            isOpen = false;
            for(Closeable closeableThread : runningThreads) {
                try {
                    closeableThread.close();
                } catch(IOException err) {
                    telemetry.addData("...wat o_O", err);
                    telemetry.update();
                }
                runningThreads.remove(closeableThread);
            }
            interrupt();
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

        /**
         * Makes the calling thread wait for the previous call of switchArmMode 
         * to have finished executing the arm switch.
         */
        public void waitForSwitch() {
            while(getIsSwitching() && isOpen) {
                // Still just waiting
            }    
        }
        
        /**
         * Makes the calling thread wait for the previous call of a extendSlides 
         * to have finished extending/retracting to the positioin.
         */
        public void waitForExtension() {
            while(getIsExtending() && isOpen) {
                // Still just waiting
            }
        }

        private synchronized void setIsExtending(boolean isExtending) {
            this.isExtending = isExtending;
        }

        private synchronized void setIsSwitching(boolean isSwitching) {
            this.isSwitching = isSwitching;
        }

        public synchronized boolean getIsExtending() {
            return this.isExtending;
        }

        public synchronized boolean getIsSwitching() {
            return this.isSwitching;
        }

        public synchronized boolean getIsWaiting() {
            return getIsExtending() || getIsSwitching();
        }
    }

    /**
     * Waits in a sepate thread until a condition is satifsied. Once the condition is true,
     */
    private class ConditionalThread extends CloseableThread {
        private BooleanSupplier condition;
        private Consumer<Boolean> onContinue = (Boolean unusedParam) -> {/* NOOP */};
        private Consumer<Boolean> onFinish;
        private boolean isOpen = true;

        public ConditionalThread() {
            super();
        }

        public ConditionalThread(BooleanSupplier condition, Consumer<Boolean> onFinish) {
            super();
            this.condition = condition;
            this.onFinish = onFinish;
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
            Consumer<Boolean> onFinish
        ) {
            this.condition = condition;
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

    /**
     * Implements constructors, methods, and attributes so that an opmode may be 
     * able to close the thread manually or automatically. These are automatically 
     * daemon threads.
     */
    // Man, I hope you like OOP programming; otherwise, this header will be a mystery.
    private abstract class CloseableThread extends Thread implements Closeable {
        public CloseableThread() {
            super();
            setDaemon(true);
        }

        public abstract void run();

        public abstract void close();
    }

    private final LiftHandlerThread lift = new LiftHandlerThread(); 

    @Override
    public void opMode_init() {
        super.opMode_init();
        
        // telemetry.addData("Real IMU heading (DEG)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.DEGREES));
        // telemetry.addData("Real IMU heading (RAD)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.RADIANS));
        // telemetry.addLine();
        // telemetry.addData("Real IMU heading Vel (DEG)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.DEGREES));
        // telemetry.addData("Real IMU heading Vel (RAD)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.RADIANS));
        // telemetry.update();
        
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
            "The robot should be positioned on the close edge of the tile " +
            "closest to the net zone (baskets area) without being inside it. The " + 
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

        // Detecting buttonPresses
        try {
            toggleBlueSide.activateIfPressed();
            toggleObservationPark.activateIfPressed();
            repositionToggle.activateIfPressed();
        } catch(IllegalAccessException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err);
        }

        if(repositionEnabled) {
            driveWheels();
        }
    }

    /**
     * Extends the slides to a length of approx 12 inches. 
     * 
     * <p> To use this as a blocking method, call afterwards 
     * lift.waitForFinish(). If that's called, this takes about 2000 ms.
     * 
     * @throws InterruptedException
     */
    private void extendAsync() throws InterruptedException {
        lift.extendSlides(TWELVE_INCHES_EXTENSION, 20, EXTENSION_POWER);
    }

    /**
     * Retracts the arm. 
     * 
     * <p> To convert this to a blocking method, call the method 
     * lift.waitForFinish(). If that's called, this takes about 2000 ms.
     * 
     * @throws InterruptedException
     */
    private void retractAsync() throws InterruptedException {
        lift.extendSlides(FULLY_RETRACTED, 20, RETRACTION_POWER);
    }

    /**
     * Attempts to move and power the intake so that a sample is grabbed. The 
     * arm must be extended to position to grab. The arm remains extended 
     * afterwards.
     * 
     * <p> Estimated asynchronous time: 500 ms
     * 
     * @throws InterruptedException
     */
    private void grabSampleAsync() throws InterruptedException {
        // lift.setIntakeToHighSpeed();
        gamepad2.right_trigger = 1.0f; // Move to intake position
        sleep(150);
        gamepad2.right_trigger = 0;
    }

    /**
     * Puts the held sample into the bucket. The arm must be in position, but 
     * the intake should be not ready. The pivot is not affected.
     * 
     * <p> Estimated async completion time: 500 ms
     * 
     * @throws InterruptedException
     */
    private void depositAsync() throws InterruptedException {
        gamepad2.right_trigger = 1.0f; // Move intake for desposit
        gamepad2.right_bumper = true; // Deposit
        sleep(150);
        gamepad2.right_trigger = 0;
        gamepad2.right_bumper = false;
    }
    
    /**
     * Extends the arm to the buckets when in deposit.
     * 
     * <p> Estimated async completion time: 3000 ms
     */
    private void extendToBucketsAsync() {
        lift.extendSlides(
           (int) AutoArmRunner.SLIDE_CONSTANTS.topBucketHeightAlternate, 
           10, 
           EXTENSION_POWER
        );
    }

    /**
     * Extends the arm and grabs the sample. The distance is fixed. The arm 
     * attempts the grab only once. The arm remains extended afterwards.
     * 
     * <p> Estimated blocking runtime: 2750 ms
     *  
     * @throws InterruptedException
     */
    private void grabSampleSync() throws InterruptedException {
        // TODO: Make this retry if nothing is grabbed
        extendAsync();
        lift.waitForFinish(); // Waiting for full extension
        grabSampleAsync(); // Grab the sample
        sleep(750); // Wait for completion
    }

    /**
     * Puts the sample in the bucket. The arm is lowered afterwards. Must be 
     * fully raised when called.
     * 
     * <p> Estimated blocking runtime: 2500 ms
     */
    private void berriddenOfSample() throws InterruptedException {
        // The arm is raised, so put it into the basket!
        depositAsync();
        sleep(500); // To stop from accidentally moving too fast
        switchArmAsync();
        lift.waitForSwitch(); // TODO: Make this so movement can begin ASAP
    }

    /**
     * Retracts the arm fully and switches the arm position. The arm becomes
     * fully retracted.
     * 
     * <p> Estimated async completion time: 2000 ms
     * 
     * @throws InterruptedException
     */
    private void switchArmAsync() throws InterruptedException {
        // Retraction is automatically handled by this method
        lift.switchArmMode();
    }

    /**
     * Moves the robot forward in the direction towards the net. Specifically, 
     * the angle is 135 deg CCW from the global x-axis.The distance traveled is 
     * approx the given dist.
     * 
     * @param dist How far to travel.
     */
    private void netMoveSync(double dist) {
        final Vector2d curPos = getCurrentPosition().position;
        final Vector2d offset = (new Vector2d(
            -dist / Math.sqrt(2), 
            dist / Math.sqrt(2)
        ));
        lineTo(
            globalDrive, 
            curPos.plus(offset) // Move forward dist inches 
        );
    }

    /**
     * Extends the linear slides so that the specimen can hook
     */
    private void extendToChambersAsync() {
        lift.extendSlides(CHAMBER_EXTENSION, 10, 1.0);
    }

    private void hookChamber() throws InterruptedException {
        retractAsync();
    }

    /**
     * Moves the robot to the chambers and places the specimen. The linear 
     * slides are in the slightly retracted position, and the pivot is in deposit.
     */
    private void moveAndPlaceSpecimen() throws InterruptedException {
        switchArmAsync();
        lift.waitForFinish();
        extendToChambersAsync();
        moveToCloseChamberInitial();
        lift.waitForFinish();
        
        // Hooking the specimen onto the chamber'
        hookChamber();
    }

    /**
     * Moves the robot beyond the submersible and to the closest spike mark.
     */
    private void clearChamberToSpikeMark() {
        Pose2d intakeOffset = new Pose2d(-6.25, 3, -Math.toRadians(90)); // Offset from bot center
        Pose2d grabbingDistance = new Pose2d(-16, 0, 0);
        final Pose2d firstSpikePosition = addPoses(
            addPoses(BLUE_NEUTRAL_TAG, new Pose2d(0, 24, 0)), // Going 1 tile from the spike mark 
            addPoses(intakeOffset, grabbingDistance) // Going to the left of the spike mark, facing it
        );
        final Action driveToFirstSpikeMark = globalDrive.actionBuilder(new Pose2d(0, 0, 0))
            .setTangent(Math.toRadians(90))
            .lineToYConstantHeading(36) // Clear the submersible.
            .splineToConstantHeading(firstSpikePosition.position, firstSpikePosition.heading)
            .build();
        Actions.runBlocking(driveToFirstSpikeMark);
    }

    // DEV START
    /**
     * Logs out the name of the section to time. The timer returned can be 
     * accessed after the action is over to tell how long it ran.
     * 
     * @param sectionName The name of the section. Is logged to telemetry, but 
     * has no bearing on the timing logic; e.g., using the same name twice still 
     * creates independent timers, but the telemetry looks the same.
     * @return A timer for logging the time
     */
    private ElapsedTime timeSection(String sectionName) {
        // telemetry.addData("Timing out section", sectionName);
        accumulated += "\nTiming out section: " + sectionName;
        // telemetry.update();
        final ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        return timer;
    }

    private void logTime(ElapsedTime timer) {
        // telemetry.addData("Seconds Elapsed", ((double) ((int) (10 * timer.seconds()))) / 10);
        accumulated += "\nSeconds Elapsed: " + ((double) ((int) (10 * timer.seconds()))) / 10;
        // telemetry.addLine();
        accumulated += "\n";
        // telemetry.update();
    }

    private String accumulated = "";
    //DEV END

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

        // DEV START starting initialization for timing
        telemetry.setAutoClear(false);
        ElapsedTime timer;
        // DEV END

        // Starting the actual stuffs
        lift.start();

        // Driving to the chamber and hooking the specimen
        timer = timeSection("chamber_inital");
        globalDrive.updatePoseEstimate();
        moveAndPlaceSpecimen();
        logTime(timer);

        // Driving to the spike marks
        boolean isFirstSpikeSample = true;
        for(int i = 2; i >= 2 && opModeIsActive(); i--) { // TODO: This only does first. Change it to adapt!
            // Initial positioning data
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final double extraRotation = i == 0 ? Math.toRadians(20) : 0; // Rotate more cuz' last one's hard to get to. 
            final Pose2d intakeOffset = new Pose2d(-6.25, 3, extraRotation - Math.PI / 2); // Offset from bot center
            final Pose2d grabbingDistance = new Pose2d(-16, 0, 0);
            
            // Finding the offset
            final Pose2d totalOffset = addPoses(intakeOffset, grabbingDistance);
            final Vector2d[] rotationMatrix = {
                new Vector2d(Math.cos(extraRotation), Math.sin(extraRotation)),
                new Vector2d(-Math.sin(extraRotation), Math.cos(extraRotation))
            };
            final Vector2d rotatedPosition = transformVector(totalOffset.position, rotationMatrix);
            final Pose2d finalOffset = new Pose2d(rotatedPosition, totalOffset.heading); 
            
            // Moving to grab the sample
            if(isFirstSpikeSample) {
                clearChamberToSpikeMark();
            } else {
                isFirstSpikeSample = false;
                setDestinationOffset(finalOffset); // Puts the robot into grabbing position
                globalDrive.updatePoseEstimate();
                moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);
            }

            // Grabbing the pixel
            resetDestinationOffset();
            
            timer = timeSection("grab_sample_" + (3 - i));
            grabSampleSync(); // Grab the pixel. and retract
            logTime(timer);

            timer = timeSection("retract_sync_" + (3 - i));
            retractAsync();
            lift.waitForFinish();
            logTime(timer);

            timer = timeSection("switch_arm_" + (3 - i));
            switchArmAsync(); // Switch the arm up; completed by travel time
            logTime(timer);
            
            // Scoring!
            globalDrive.updatePoseEstimate();

            timer = timeSection("move_zone_" + (3 - i));
            moveRobotToNetZone(isBlue);
            logTime(timer);

            timer = timeSection("arm_raise_" + (3 - i));
            extendToBucketsAsync();
            lift.waitForFinish();
            logTime(timer);

            timer = timeSection("move_forward_" + (3 - i));
            netMoveSync(8); // inches 
            logTime(timer);
            
            timer = timeSection("berrideden_of_sample_" + (3 - i));
            berriddenOfSample();
            logTime(timer);
        }

        // lift.close();

        telemetry.clearAll();
        telemetry.addLine(accumulated);
        telemetry.update();

        // Putting the arm at position zero for the teleop folks
        AutoInit.driveMotorTo(
            linearSlidePivot, 
            PIVOT_MIN_POSITION, 
            3, 
            -0.5
        );

        /*// Parking
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
        }*/

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
