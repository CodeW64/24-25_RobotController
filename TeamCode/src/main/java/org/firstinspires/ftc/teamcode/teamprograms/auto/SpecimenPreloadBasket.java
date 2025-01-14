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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
// import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.function.DoubleConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

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
         24 - ROBOT_CENTER.position.y, //  0 is the top y-coord
         0  + ROBOT_CENTER.heading.toDouble() // 0 is the deafult rotation
    );

    private double sampleSensingDistance;

    private int CHAMBER_EXTENSION = 2500;
    private int EXTEND_TO_SAMPLE_EXTENSION = 1377;
    private int FULLY_RETRACTED = 0;
    
    private double EXTENSION_POWER = 1.0; // Previously 0.15
    private double RETRACTION_POWER = -1.0; // Previous -0.4
    private boolean isTime = false; // DEV: This is exists for debuggin telemetry

    public static interface ThreadIdentifiers {
            public static enum Type { UNKNOWN, EXTENSION, SWITCH }

            public final static Type UNKNOWN = Type.UNKNOWN;
            public final static Type EXTENSION = Type.EXTENSION;
            public final static Type SWITCH = Type.SWITCH;

            public Type getType();

            public String getName();
    }

    /**
     * Contains methods so that the arm can be managed from outside the teleop, 
     * asynchronously.
     */
    private class LiftHandlerThread extends CloseableThread {
        private boolean isOpen = true;
        private Boolean isExtending = false;
        private Boolean isSwitching = false;
        private boolean hasStartedSwitch = true;
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
         * @param power How powerful the motor should be run. 
         */
        public void extendSlides(int target, int tolerance, double power) {
            if(getIsExtending()) {
                // Closing (canceling) any other extension threads to prevent race conditions and memory leaks
                for(final ConditionalThread thread : runningThreads) {
                    if(thread.identifiers.getType() == ThreadIdentifiers.EXTENSION) {
                        thread.close();
                        runningThreads.remove(thread);
                    }
                }
            }
            
            setIsExtending(true);
            // TODO: Add id's to the threads? 
            final ConditionalThread extensionThread = new ConditionalThread(new ThreadIdentifiers() {
                public Type getType() {
                   return ThreadIdentifiers.EXTENSION; 
                }

                public String getName() {
                    return "Extension.extensionThread";
                }
            });
            
            extensionThread.finishInitialization(
                () -> Math.abs(getLinearSlideAvgPosition() - target) <= tolerance,
                (Boolean unusedParam) -> {
                    // AutoInit.driveMotorTo(linearSlideLift, target, tolerance, power);
                    linearSlideLeft.setTargetPosition(target);
                    linearSlideLeft.setTargetPositionTolerance(tolerance);
                    linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    linearSlideLeft.setPower(power);

                    linearSlideRight.setTargetPosition(target);
                    linearSlideRight.setTargetPositionTolerance(tolerance);
                    linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    linearSlideRight.setPower(power);
                },
                (Boolean unusedParam) -> {
                    linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    linearSlideLeft.setPower(0);
                    linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    linearSlideRight.setPower(0);
                    setIsExtending(false);
                    runningThreads.remove(conditionalThread);
                }
            );
            runningThreads.add(extensionThread);
            extensionThread.start();
        }

        /**
         * Returns whether the given state of the lift is a despoit-position 
         * state. This includes exteneded, extended alternate, and retracting.
         * 
         * @param state The state to check the position of.
         * @return Boolean describing wether it is accurate to call the state a 
         *     deposit state.
         */
        public boolean isDepositPosition(AutoArmRunner2.LinearSlideStates state) {
            return state == AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE
                || state == AutoArmRunner2.LinearSlideStates.PIVOT_TO_DEPOSIT
                || state == AutoArmRunner2.LinearSlideStates.DEPOSIT_RETRACT
                || state == AutoArmRunner2.LinearSlideStates.DEPOSIT_RETRACT_SET;
        }
        
        /**
         * Returns whether the pivot has ended pivoting. It checks by seeing if 
         * it landed in an end-pivot slides state.
         * 
         * @return Whether the slides have fully pivoted
         */
        public boolean hasFinishedPivot() {
            return linearSlideState.equals(AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE)
                || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_ACTIVE)
                || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_FULL);
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchArmMode() {
            if(getIsSwitching()) {
                // Closing (canceling) any other extension threads to prevent race conditions and memory leaks
                for(final ConditionalThread thread : runningThreads) {
                    if(thread.identifiers.getType() == ThreadIdentifiers.SWITCH) {
                        thread.close();
                        runningThreads.remove(thread);
                    }
                }
            }

            // SEt and set initial conditions
            final AutoArmRunner2.LinearSlideStates intialState = linearSlideState; 
            final boolean initialStateIsDeposit = isDepositPosition(linearSlideState);
            setIsSwitching(true);
            hasStartedSwitch = true;
            
            // Initialize the process for switiching
            // TODO: Add id's to the threads? 
            final ConditionalThread buttonPresser = new ConditionalThread("switchArmMode.buttonPresser", ThreadIdentifiers.SWITCH);
            final ConditionalThread armSwitcher = new ConditionalThread("switchArmMode.armSwitcher", ThreadIdentifiers.SWITCH);

            buttonPresser.finishInitialization(
                () -> isDepositPosition(linearSlideState) != initialStateIsDeposit, 
                (Boolean unusedParam) -> gamepad2.left_trigger = 1.0f, 
                (Boolean unusedParam) -> {
                    gamepad2.left_trigger = 0;
                    runningThreads.remove(buttonPresser);
                    hasStartedSwitch = false;
                    armSwitcher.start();
                }
            );

            armSwitcher.finishInitialization(
                () -> hasFinishedPivot(),
                (Boolean unusedParam) -> {
                    runningThreads.remove(armSwitcher);
                    setIsSwitching(false);
                } 
            );

            // Starting the processes
            runningThreads.add(buttonPresser);
            runningThreads.add(armSwitcher);
            buttonPresser.start();
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchToChamber() {
            // Set initial conditions
            setIsSwitching(true);
            hasStartedSwitch = false;
            
            // Initialize the process for switiching
            // TODO: Add id's to the threads? 

            final ConditionalThread buttonPresser = new ConditionalThread("switchToChamber.buttonPresser", ThreadIdentifiers.SWITCH);
            final ConditionalThread armSwitcher = new ConditionalThread("switchToChamber.armSwithcer", ThreadIdentifiers.SWITCH);

            buttonPresser.finishInitialization(
                () -> 
                    linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_POSITION) 
                    && Math.abs(getLinearPivotAvgPosition() - PIVOT_CONSTANTS.specimenPositionPos) < 20, 
                (Boolean unusedParam) -> {
                    gamepad2.left_trigger = 0;
                    gamepad2.dpad_left = false;
                    runningThreads.remove(buttonPresser);
                    hasStartedSwitch = true;
                    armSwitcher.start();
                }
            );

            armSwitcher.finishInitialization(
                () -> hasFinishedPivot(),
                (Boolean unusedParam) -> {
                    runningThreads.remove(armSwitcher);
                    setIsSwitching(false);
                } 
            );

            // Starting the processes
            runningThreads.add(buttonPresser);
            runningThreads.add(armSwitcher);
            gamepad2.left_trigger = 1.0f;
            gamepad2.dpad_left = true;
            buttonPresser.start();
        }

        public void print() {
            telemetry.addLine("---------- Lift Handler Threads --------");
            telemetry.addData("All running threads", runningThreads.size());
            telemetry.addLine("All running threads:");

            // logging all the data
            for(final ConditionalThread thread : runningThreads) {
                telemetry.addLine("  " + thread.identifiers.getName());
            }
        }

        @Override
        public void close() {
            isOpen = false;
            for(Closeable closeableThread : runningThreads) {
                try {
                    closeableThread.close();
                } catch(IOException err) {
                    telemetry.addData("...wat o_O", err.getMessage());
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
        public void waitForFinish() throws InterruptedException {
            while(getIsWaiting() && isOpen) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }
        }

        /**
         * Makes the calling thread wait for the previous call of switchArmMode 
         * to have finished executing the arm switch.
         */
        public void waitForSwitch() throws InterruptedException {
            while(getIsSwitching() && isOpen) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }    
        }
        
        /**
         * Makes the calling thread wait for the previous call of a extendSlides 
         * to have finished extending/retracting to the positioin.
         */
        public void waitForExtension() throws InterruptedException {
            while(getIsExtending() && isOpen) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }
        }

        /**
         * Makes the calling thread wait for the previous call of switchArmMode 
         * to have registered the button press and started the switch
         */
        public void waitForSwitchStart() throws InterruptedException {
            while(!getHasStartedSwitch() && isOpen) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
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

        public synchronized boolean getHasStartedSwitch() {
            return hasStartedSwitch;
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
        public ThreadIdentifiers identifiers;

        public ConditionalThread() {
            super();
        }
        
        public ConditionalThread(String name, ThreadIdentifiers.Type type) {
            super();
            this.identifiers = new ThreadIdentifiers() {
                public ThreadIdentifiers.Type getType() {
                    return type;
                }

                public String getName() {
                    return name;
                }
            };
        }

        public ConditionalThread(ThreadIdentifiers identifiers) {
            super();
            this.identifiers = identifiers;
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
            try {
                while(currentBoolean = (!condition.getAsBoolean() && isOpen)) {
                    onContinue.accept(currentBoolean);
                    Thread.sleep(30); // Allow for the process in other threads to continue;
                }

                onFinish.accept(!condition.getAsBoolean());
            } catch(InterruptedException err) {
                telemetry.addData("Interupted Running Conditional Thread: ", err.getMessage());
            }
        }
    
        @Override
        public void close() {
            isOpen = false;
            interrupt();
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
        isTelemetrySuppresed = true;
        
        // Initializing other hardware(-ish) bits
        globalDrive = new MecanumDrive(hardwareMap, START_LOCATION);
        sampleSensingDistance = hardwareMap
            .get(ColorRangeSensor.class, "sampleSensor")
            .getDistance(DistanceUnit.CM);

        // lift.start();
        // initHardware();
        

        // Creating init_loop options
        try {
            toggleBlueSide = new ButtonPressHandler(gamepad1, "a", (Gamepad g) -> {
                isBlue = !isBlue;
                neutralTagId = isBlue ? AprilLocater.NEUTRAL_BLUE_ID : AprilLocater.NEUTRAL_RED_ID;
                coloredTagId = isBlue ? AprilLocater.COLORED_BLUE_ID : AprilLocater.COLORED_RED_ID;
            });
            
            repositionToggle = new ButtonPressHandler(gamepad1, "start", (Gamepad g) -> {
                repositionEnabled = !repositionEnabled;
            });
        } catch(NoSuchFieldException | NullPointerException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err.getMessage());
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
        telemetry.addData("Respoition Toggle", "Press " + repositionToggle.getButtonName());

        // Detecting buttonPresses
        try {
            toggleBlueSide.activateIfPressed();
            repositionToggle.activateIfPressed();
        } catch(IllegalAccessException err) {
            telemetry.addData("!!CAUGHT BUTTON ERROR", err.getMessage());
        }

        if(repositionEnabled) {
            driveWheels();
        }
    }

    private void driveLiftTo(int target, int tolerance, double power) {
        while(Math.abs(getLinearSlideAvgPosition() - target) > tolerance) {
            // AutoInit.driveMotorTo(linearSlideLift, target, tolerance, power);
            linearSlideLeft.setTargetPosition(target);
            linearSlideLeft.setTargetPositionTolerance(tolerance);
            linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(power);

            linearSlideRight.setTargetPosition(target);
            linearSlideRight.setTargetPositionTolerance(tolerance);
            linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideRight.setPower(power);
        }

        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideLeft.setPower(0);
        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideRight.setPower(0);
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
        lift.extendSlides(EXTEND_TO_SAMPLE_EXTENSION, 10, EXTENSION_POWER);
    }

    private void extendSync(int offset) {
        final int target = EXTEND_TO_SAMPLE_EXTENSION + offset;
        final int tolerance = 20;
        final double power = EXTENSION_POWER;
        driveLiftTo(target, tolerance, power);
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
        lift.extendSlides(FULLY_RETRACTED, 30, RETRACTION_POWER);
    }

    private void retractSync() {
        final int target = FULLY_RETRACTED;
        final int tolerance = 50;
        final double power = EXTENSION_POWER;
        driveLiftTo(target, tolerance, power);
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
        // Presssing the grab button
        linearSlideLeft.setPower(0);
        linearSlideRight.setPower(0);
        isStateInitialized = false;
        linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
        intakeWheelR.setPower(INTAKE_POWER_HOLD);
        intakeWheelL.setPower(INTAKE_POWER_HOLD);
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
        intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
        sleep(400);
        intakeWheelR.setPower(INTAKE_POWER_EMPTY);
        intakeWheelL.setPower(INTAKE_POWER_EMPTY);
    }
    
    /**
     * Extends the arm to the buckets when in deposit.
     * 
     * <p> Estimated async completion time: 3000 ms
     */
    private void extendToBucketsAsync() {
        lift.extendSlides(
            (int) AutoArmRunner2.SLIDE_CONSTANTS.topBucketHeightAlternate, 
            10, 
            EXTENSION_POWER
        );
    }

    private void extendToBucketsSync() {
        final int target = (int) AutoArmRunner2.SLIDE_CONSTANTS.topBucketHeightAlternate;
        final int tolerance = 10;
        final double speed = EXTENSION_POWER;
        driveLiftTo(target, tolerance, speed);
    }

    /**
     * Extends the arm and grabs the sample. The distance is fixed. The arm 
     * attempts the grab only once. The arm remains extended afterwards.
     * 
     * <p> Estimated blocking runtime: 2750 ms
     *  
     * @throws InterruptedException
     */
    private void grabSampleSync(boolean retry, int offset) throws InterruptedException {
        // TODO: Make this retry if nothing is grabbed
        telemetry.addLine("Extending to sample...");
        telemetry.update();
        lift.waitForFinish(); // Wait for the arm to finish retraction
        extendSync(offset);
        // lift.waitForExtension(); // Waiting for full extension

        retryLoop:
        while(!isPossessingSample(currentSampleDistance)) {
            telemetry.addLine("Switching to intake mode to grab...");
            telemetry.update();
            grabSampleAsync(); // Grab the sample

            // Wait for completion
            telemetry.addLine("Sleeping");
            telemetry.update();
            // CAPUT MEUM DOLET.
            while(
                !linearSlideState.equals(LinearSlideStates.INTAKE_FULL) 
                && !linearSlideState.equals(LinearSlideStates.INTAKE_EMPTY)
                && !linearSlideState.equals(LinearSlideStates.INTAKE_ACTIVE)
            ) {
                sleep(30); // Waiting whilst freeing CPU for other threads
            }

            // Exiting the retry loop if we don't want to retry
            if(!retry) {
                break retryLoop; 
            }

            // Moving so that we have more chance of getting it.
            setDestinationOffset(new Pose2d(1, 2, 0)); // Move from current position forward and a sample up
            lineTo(globalDrive, getCurrentPosition());
            resetDestinationOffset();
        }
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

        // Protecting the intake
        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
        sleep(300);

        // Lowering and switching
        switchArmAsync();
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
        lift.extendSlides(CHAMBER_EXTENSION, 20, 1.0);
    }

    private void hookChamber() throws InterruptedException {
        retractAsync();
    }

    private void pivotDown() throws InterruptedException {
        final int target = (int) PIVOT_CONSTANTS.specimenPlacePos;
        final int tolerance = 20;

        while(Math.abs(getLinearPivotAvgPosition() - target) <= tolerance) {
            linearPivotTargetPosition = target;
        }
    }

    /**
     * Moves the robot to the chambers and places the specimen. The linear 
     * slides are in the slightly retracted position, and the pivot is in deposit.
     */
    private void moveAndPlaceSpecimen() throws InterruptedException {
        // switchArmAsync();
        // extendToChambersAsync();
        // moveToCloseChamberInitial(distFromChamber);
        // lift.waitForSwitch();

        lineTo( globalDrive, addPoses(getCurrentPosition(), new Pose2d(8, 0, 0)) );
        lift.switchToChamber();
    
        // Wait for the pivot to be reasonably rotated before extending 
        final double bound = (PIVOT_CONSTANTS.specimenPositionPos + PIVOT_CONSTANTS.minPos) / 2;
        while(getLinearPivotAvgPosition() < bound) {
            sleep(30); // Give time to other threads to do their thang
        }

        // Extend, move, and wait for the switch before hooking
        final double SAFETY_DIST = 24; // Used to prevent contact with the 
        final double ALLIANCE_SHARING_DIST = 2; // Inches from the tile teeth, for space
        extendToChambersAsync();
        // lineTo(globalDrive, new Pose2d(
        //     BLUE_CHAMBER.position.x - SAFETY_DIST, 
        //     BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y, 
        //     0
        // ));
        final Pose2d currentPos = getCurrentPosition();
        final Pose2d dest = new Pose2d(
            BLUE_CHAMBER.position.x - SAFETY_DIST, 
            BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y + ALLIANCE_SHARING_DIST, 
            0
        );
        Actions.runBlocking(globalDrive.actionBuilder(currentPos)
            .setTangent(Math.atan2(currentPos.position.y - dest.position.y, currentPos.position.x - dest.position.x))
            .lineToY(dest.position.y, new TranslationalVelConstraint(20.0))
            .build()
        );
        resetDestinationOffset();
        lift.waitForFinish();

        
        // Moving forward and hooking onto the chamber'
        // lineTo(globalDrive, BLUE_BUCKETS_CHAMBER);
        pivotDown(); // Lowering the motor to prevent collisions
        hookChamber();
        
        // Waiting for the hook to fully... well, hook.
        while(getLinearSlideAvgPosition() >= FULLY_RETRACTED + 30) {
            sleep(30); // Wait whiling freeing up CPU for other threads.
        }
    }

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

    /**
     * Executes the main code for the robot. This exists so the code isn't 
     * polluted with try-catches, but can be declared entirely. If an exception 
     * is thrown, it would only be reasonable in the case of op mode end, where 
     * we want all function to end anyways. 
     * 
     * @throws InterruptedException
     */
    private void main(boolean arg) throws InterruptedException {
        // Completely disabling the idea of hitting buttons
        toggleBlueSide = null;
        toggleObservationPark = null;
        repositionToggle = null;

        // DEV START starting initialization for timing
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        ElapsedTime timer;
        // DEV END

        // Starting the actual stuffs
        lift.start();
        SLIDE_CONSTANTS.depositEndRetract = EXTEND_TO_SAMPLE_EXTENSION;
        SLIDE_CONSTANTS.intakeEndRetract = EXTEND_TO_SAMPLE_EXTENSION;

        // Driving to the chamber and scoring
        timer = timeSection("chamber_inital");
        globalDrive.updatePoseEstimate();
        // globalDrive.maxWheelVel = 25;
//        if(!arg) {
//        moveAndPlaceSpecimen();
//        switchArmAsync(); // Lowring the arm
//        // globalDrive.maxWheelVel = 40;
//        logTime(timer);
//        }
        // moveAndPlaceSpecimen();
        // switchArmAsync(); // Lowring the arm
        // globalDrive.PARAMS.maxWheelVel = 30;
        // logTime(timer);
        // if(arg) {
        //     return;
        // }

        // Driving to the spike marks
        boolean isFirstSpikeSample = true;
        for(int i = 2; i >= 0 && opModeIsActive(); i--) {
            // Initial positioning data
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final double extraRotation = i == 0 ? Math.toRadians(30) : 0; // Rotate more cuz' last one's hard to get to. 
            final Pose2d intakeOffset = new Pose2d(-6.25, 1.5, extraRotation - Math.PI / 2); // Offset from bot center
            final Pose2d grabbingDistance = new Pose2d(-20, 0, 0);
            
            // Finding the offset
            final Pose2d totalOffset = addPoses(intakeOffset, grabbingDistance);
            final Vector2d[] rotationMatrix = {
                // __          __ 
                // |  M11  M12  | 
                // |_ M21  M22 _| 
                new Vector2d(/*M11*/Math.cos(extraRotation),  /*M21*/Math.sin(extraRotation)),
                new Vector2d(/*M12*/-Math.sin(extraRotation), /*M22*/Math.cos(extraRotation))
            };
            final Vector2d rotatedPosition = transformVector(totalOffset.position, rotationMatrix);
            final Pose2d finalOffset = new Pose2d(rotatedPosition, totalOffset.heading); 
            
            // Moving to grab the sample
            setDestinationOffset(finalOffset); // Puts the robot into grabbing position
            globalDrive.updatePoseEstimate();
            telemetry.addLine("\n=====> Moving to the spike mark...");
            telemetry.update();
            isTime = true;
            setRotationType(RotationType.SPLINE);
            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            // if(getCurrentPosition().heading.toDouble()) {

            // }

            // Grabbing the pixel
            resetDestinationOffset();
            timer = timeSection("grab_sample_" + (3 - i));
            telemetry.addLine("\n=====> Grabbing the sample...");
            telemetry.update();
            grabSampleSync(false, -50 * (2 - i)); // Grab the pixel. and retract
            logTime(timer);
            isTime = false;

            timer = timeSection("retract_sync_" + (3 - i));
            retractSync();
            // lift.waitForFinish();
            logTime(timer);

            timer = timeSection("switch_arm_" + (3 - i));
            switchArmAsync(); // Switch the arm up; completed by travel time
            logTime(timer);
            
            // Scoring!
            globalDrive.updatePoseEstimate();

            timer = timeSection("move_zone_" + (3 - i));
            final double DIST_INCREMENT = 0.5; // NOTE: change this when roadRunner is tuned
            final double DIST_BACK = 9.5/*  - DIST_INCREMENT * (2 - i) */;
            final double DIST_STRAFE = 2.5;            
            final double SQRT2 = Math.sqrt(2);
            final Pose2d BACK_AWAY = new Pose2d((DIST_BACK + DIST_STRAFE) / SQRT2, (DIST_STRAFE - DIST_BACK) / SQRT2, 0); // don't go too close to the buckets
            setDestinationOffset(BACK_AWAY); // Move back 4 inches to avoid accidental hanging
            moveRobotToNetZone(isBlue);
            resetDestinationOffset();
            logTime(timer);

            
            // Waiting for the pivot to get up before extendning
            // final double PIVOT_BUCKET_SAFETY = (PIVOT_ALTERNATE_DEPOSIT_POSITION + PIVOT_MIN_POSITION) / 2;
            lift.waitForSwitch();
            
            // Extending to the buckets
            timer = timeSection("arm_raise_" + (3 - i));
            extendToBucketsSync();
            lift.waitForFinish();
            logTime(timer);

            // timer = timeSection("move_forward_" + (3 - i));
            // netMoveSync(8); // inches 
            // logTime(timer);
            
            timer = timeSection("berrideden_of_sample_" + (3 - i));
            berriddenOfSample();
            logTime(timer);
        }

        // Retracting fully after the last basket
        driveLiftTo(0, 10, RETRACTION_POWER);

        // lift.close();

        // telemetry.clearAll();
        telemetry.addLine(accumulated);
        telemetry.update();

        // // Parking
        // telemetry.addData("Status", "Completed!");
        // telemetry.update();

        // final AprilTagDetection observationZone = getDetection(this.coloredTagId);
        // if(this.shouldParkObservation) {
        //     globalDrive.updatePoseEstimate();
        //     moveRobotToObservation(observationZone, false); 
        // } else {
        //     globalDrive.updatePoseEstimate();
        //     moveRobotToAscentZone(observationZone);
        //     globalDrive.updatePoseEstimate();
        //     attemptAscent(1);
        // }

        // Putting the arm at position zero for the teleop folks
        gamepad1.dpad_left = true;
        gamepad2.dpad_left = true;

        // Yipee! Finishing up
        telemetry.addData("Status", "Completed! 🥳");
        telemetry.update();
    }
    // private void main(boolean arg) throws InterruptedException {
    //     lift.start();
    //     sleep(5000);
    //     isTime = true;
    //     grabSampleSync(); // Grab the pixel. and retract 
    //     isTime = false;
    //     throw new InterruptedException();
    // }

    @Override
    public void opMode_start() {
        try {
            main(true);
        } catch(InterruptedException err) {
            telemetry.addData("!! CAUGHT FATAL ERROR IN MAIN", err.getMessage());
            telemetry.update();
            // sleep(5000);
            // telemetry.clear();
        }
    }

    @Override
    public void opMode_stop() {
        powerDriveMotors(0, 0, 0, 0);
        lift.close();
    }
}
