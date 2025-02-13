package org.firstinspires.ftc.teamcode.teamprograms.auto;


import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;

import java.io.Closeable;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.ConcurrentModificationException;
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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.ftc.Actions;

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
@Config
@Autonomous(name="Observation Path (Spike Mark Pusher ~ 2 Push)")
public class ObservationPath extends AutoCommonPaths {
    private boolean isBlue = true;
    private boolean shouldParkObservation = true; // False: Go to ascent zone there
    private int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    private int coloredTagId = AprilLocater.COLORED_BLUE_ID;
    private boolean repositionEnabled = false;
    // DEV: This line is true to do testing; please make it false by thurs
    public static boolean isTeleopMode = false; // FIXME: MAKE SURE THIS IS ____NEVER___ ENABLED OR ALLOWED IN A MATCH
    public static boolean isDebugMode = false;

    private ButtonPressHandler toggleBlueSide;
    private ButtonPressHandler toggleObservationPark;
    private ButtonPressHandler repositionToggle;
    private ButtonPressHandler teleopModeToggle; // FIXME: MAKE SURE THIS IS ____NEVER___ ENABLED OR ALLOWED IN A MATCH

    private int extendToSampleExtension = EXTEND_TO_SAMPLE_EXTENSION;

    /**
     * Describes the robot's relative coordinates. That is, the x is width on 
     * the global field plane, y is the height, and heading is the CCW rotation 
     * from the x-axis.
     */
    private static final Pose2d ROBOT_INIT_POSE = new Pose2d(16.5, 17, Math.toRadians(0));
    
    /**
     * Describes the the center of the robot's relative coordinates. See 
     * ROBOT_INIT_POSE for an explanatiom
     */
    private static final Pose2d ROBOT_CENTER = new Pose2d(
        ROBOT_INIT_POSE.position.times(0.5),
        ROBOT_INIT_POSE.heading
    );

    /**
     * Global coordinates of where and how the robot starts for this opmode.
     */
    private static final Pose2d START_LOCATION = new Pose2d(
        -72 + ROBOT_CENTER.position.x, // -72 is the left grid x-coord 
        -24 + ROBOT_CENTER.position.y, //  0 is the top y-coord
        Math.toRadians(0)
    );

    private double sampleSensingDistance;

    public static int GRAB_FIRST = 0;
    public static int PUSH_ITER = 2;
    public static int GRAB_ITER = 2;

    public static double DIST_INCREMENT = 10;
    public static double SPECIMEN_LATERAL_INCREMENT = 1.5;
    public static double DIST_FROM = 5 + ROBOT_CENTER.position.x;
    public static double CAPTURE_BREADTH = 4;
    public static double JUST_MISSED_IT_DIST = 10;
    public static double SPIKE_MARK_LENGTH = 3.5;
    public static double SPIKE_OFFSET = 20 - SPIKE_MARK_LENGTH / 2;
    public static double BLUE_SPIKE_X = BLUE_COLORED_TAG.position.x + SPIKE_OFFSET;
    public static double END_X = -51;
    public static double SAFE_PLACE_DIST = 6;

    public static double ALLIANCE_SHARING_DIST_REPEATED = -2; // Inches from the tile teeth, for space

    public static double MAX_NET_VEL = 40;
    public static double MIN_NET_ACCEL = 50;
    public static double MAX_NET_ACCEL = 50;
    
    public static int SPECIMEN_GRAB_MS = 200;
    public static double SECONDS_CLOSE_TIMEOUT = 0.5;
    public static double GRAB_MIN_REMAINING_SEC = 4.0;
    public static double GRAB_SYNC_MAX_SEC = 5.0;
    public static double GRAB_RETRY_SEC = 1.5;
    public static double MIN_REAMINING_SCORE_SEC = 3.0;
    public static double GRAB_PAUSE = 0;

    public static int CHAMBER_EXTENSION = 1300;
    public static int EXTEND_TO_SAMPLE_EXTENSION = 1500;
    public static int EXTEND_TO_SAMPLE_LAST_EXTENSION = 1300;
    public static int FULLY_RETRACTED = 500;
    public static int HOOK_RETRACTION = 300;
    public static int GRAB_EXTENSION_TOLERANCE = 30;
    public static int RETRACTION_TOLERANCE = 40;
    public static int BUCKETS_EXTENSION_TOLERANCE = 10;
    
    public static double NET_ZONE_TOLERANCE = 0.7;

    public static double SQRT2 = Math.sqrt(2); // approx 1.4142
    public static double SPECIMEN_PLACE_OFFSET = 0;
    
    // TODO: These are not able to be edited by Dash. Make them constructed when needed?
    //       Another thought: Make a class "PoseInit" with x, y, and theta attributes,
    //       and construct the Pose2ds at runtime? Call a method on the PoseInits to convert
    //       to a Pose2d? Food for thought. 🍗
    public static Pose2d RETRY_OFFSET = new Pose2d(1.0, 0.5, 0);
    public static Pose2d INTAKE_OFFSET = new Pose2d(-6.25, 0, -Math.PI / 2);
    public static Vector2d GRABBING_DISTANCE = new Vector2d(-EXTEND_TO_SAMPLE_EXTENSION / LIFT_TICKS_PER_INCH_EXTENDED, 0);
    public static Pose2d NORMAL_GRAB_OFFSET = new Pose2d(1, -0.25, 0);
    public static double LAST_SPIKE_ANGLE = Math.toRadians(90);

    public static int MAX_GRAB_INDEX = 2;
    public static int MIN_GRAB_INDEX = 1;
    
    public static double PIVOT_SAFE_FOR_EXTENSION = 60 * PIVOT_TICKS_PER_DEGREE; // Pivot pos when extension to buckets may occur
    public static double SPECIMEN_SAFE_FOR_EXTENSION = 1280; // Pivot pos when extension to buckets may occur
    public static boolean USE_PIVOT_TOLERANCE = true; // Whether to use the fancy math for hasFinishedPivot
    public static double ARM_TO_BASKET_TOLERANCE_INCHES = 0.7; // Max arm displacement allowed when depositing
    

    /**
     * Calculates the tolerance of the pivot based on the static basket 
     * tolerance field. This allows for the lift handler to know when the pivot
     * is in a safe enough position to extend the arm. 
     * 
     * The calculation is done by using the Law of Consines and solving for the
     * angle. The arm length at the basket is used as the two adjacent sides, 
     * and the opposite side is equal to the basket tolerance. A conversion 
     * factor is then applied to the resulting angle to convert to pivot ticks
     * 
     * @return Number of pivot ticks the arm must deviate before it exceeds the 
     *     displacement tolerance described by ARM_TO_BASKET_TOLERANCE_INCHES
     */
    private static double getPivotTolerance() {
        final double INITIAL_ARM_LENGTH = 14.2;
        final double LENGTH_TO_BASKET = 
            SLIDE_CONSTANTS.topBucketHeightAlternate / LIFT_TICKS_PER_INCH_EXTENDED 
            + INITIAL_ARM_LENGTH;  
        /*
                       _            Solve for θ, where...
                      / \           
                    /    \          a = length_to_basket
               a  /       \  c      b = length_to_basket
                / )_  θ    \        c = arm_to_basket_tolerance_inches
              /_____)_______\       
                     b
         */
        return PIVOT_TICKS_PER_RAD * Math.acos(
            (
                  LENGTH_TO_BASKET * LENGTH_TO_BASKET
                + LENGTH_TO_BASKET * LENGTH_TO_BASKET
                - ARM_TO_BASKET_TOLERANCE_INCHES * ARM_TO_BASKET_TOLERANCE_INCHES
            )
            / (2 * LENGTH_TO_BASKET * LENGTH_TO_BASKET)
        );
    }

    public static double EXTENSION_POWER = 1.0; // Previously 0.15
    public static double RETRACTION_POWER = -1.0; // Previous -0.4

    public static boolean DO_TURN_DEPO = false; // Should the robot turn the pivot when depositing?
    public static boolean goToAscent = false;

    public static interface ThreadIdentifiers {
            public static enum Type { UNKNOWN, EXTENSION, SWITCH, QUE }

            public final static Type UNKNOWN = Type.UNKNOWN;
            public final static Type EXTENSION = Type.EXTENSION;
            public final static Type SWITCH = Type.SWITCH;
            public final static Type QUE = Type.QUE;

            public Type getType();

            public String getName();
    }

    /**
     * Contains methods so that the arm can be managed from outside the teleop, 
     * asynchronously.
     */
    protected class LiftHandlerThread extends CloseableThread {
        private boolean isOpen = true;
        private Boolean isExtending = false;
        private Boolean isSwitching = false;
        private boolean hasStartedSwitch = true;
        private long pressDuration = 30; // In milliseonds
        private ArrayList<ConditionalThread> runningThreads = new ArrayList<ConditionalThread>();

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
                for(int i = 0; i < runningThreads.size(); i++) {
                    final ConditionalThread thread = runningThreads.get(i);
                    if(thread.identifiers.getType() == ThreadIdentifiers.EXTENSION) {
                        thread.close();
                        runningThreads.remove(thread);
                        i--;
                    }
                }
            }
            
            setIsExtending(true);
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
                    // driveMotorToTo(linearSlideLift, target, tolerance, power);
                    setFightingGravity(false);
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
                    setFightingGravity(true);
                    linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    linearSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    linearSlideLeft.setPower(0);
                    linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    linearSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    linearSlideRight.setPower(0);
                    setIsExtending(false);
                    runningThreads.remove(extensionThread);
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
            double pivotTolerance = Double.POSITIVE_INFINITY; // No care over toleraence

            // Changing the pivot tolerance to reflect state and desiredTolerance
            if(USE_PIVOT_TOLERANCE && (
                linearSlideState.equals(AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE)
                || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_POSITION)
                || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_PLACE)
            )) {
                pivotTolerance = getPivotTolerance();
            }

            if(USE_PIVOT_TOLERANCE && (
                linearSlideState.equals(LinearSlideStates.SPECIMEN_GRAB)
            )) {
                pivotTolerance = 5;
            }

            return 
                Math.abs(getLinearPivotAvgPosition() - linearPivotTargetPosition) <= pivotTolerance
                && (
                    linearSlideState.equals(AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_ACTIVE)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_FULL)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_POSITION)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_GRAB)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.SPECIMEN_PLACE)
                );
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchArmMode() {
            if(getIsSwitching()) {
                // Closing (canceling) any other extension threads to prevent race conditions and memory leaks
                for(int i = 0; i < runningThreads.size(); i++) {
                    final ConditionalThread thread = runningThreads.get(i);
                    if(thread.identifiers.getType() == ThreadIdentifiers.SWITCH) {
                        thread.close();
                        runningThreads.remove(thread);
                        i--;
                    }
                }
            }

            // SEt and set initial conditions
            final AutoArmRunner2.LinearSlideStates intialState = linearSlideState; 
            final boolean initialStateIsDeposit = isDepositPosition(linearSlideState);
            setIsSwitching(true);
            hasStartedSwitch = true;
            
            // Initialize the process for switiching
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
            isPreloaded = false;
            runningThreads.add(buttonPresser);
            runningThreads.add(armSwitcher);
            buttonPresser.start();
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchToChamber() {
            if(getIsSwitching()) {
                // Closing (canceling) any other extension threads to prevent race conditions and memory leaks
                for(int i = 0; i < runningThreads.size(); i++) {
                    final ConditionalThread thread = runningThreads.get(i);
                    if(thread.identifiers.getType() == ThreadIdentifiers.SWITCH) {
                        thread.close();
                        runningThreads.remove(thread);
                        i--;
                    }
                }
            }

            // SEt and set initial conditions
            final AutoArmRunner2.LinearSlideStates intialState = linearSlideState; 
            setIsSwitching(true);
            hasStartedSwitch = true;
            
            // Initialize the process for switiching
            final ConditionalThread buttonPresser = new ConditionalThread("switchToChamber.buttonPresser", ThreadIdentifiers.SWITCH);
            final ConditionalThread armSwitcher = new ConditionalThread("switchToChamber.armSwitcher", ThreadIdentifiers.SWITCH);

            buttonPresser.finishInitialization(
                () -> linearSlideState == LinearSlideStates.PIVOT_TO_SPECIMEN 
                    || linearSlideState.equals(LinearSlideStates.SPECIMEN_POSITION), 
                (Boolean unusedParam) -> gamepad2.left_bumper = true, 
                (Boolean unusedParam) -> {
                    gamepad2.left_bumper = false;
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
                    isPreloaded = false;
                } 
            );

            // Starting the processes
            isPreloaded = true;
            runningThreads.add(buttonPresser);
            runningThreads.add(armSwitcher);
            buttonPresser.start();
        }

        /**
         * Toggles the arm between an intake position and a deposit position.
         */
        public void switchToSpecimenGrab() {
            // DEV START: Throw exception if not in a valid state
            if(linearSlideState != LinearSlideStates.SPECIMEN_POSITION && linearSlideState != LinearSlideStates.SPECIMEN_PLACE) {
                throw new RuntimeException("LiftHandlerThread.switchToSpecimenGrab was called when not in POSITION or PLACE state.");
            }
            // DEV END

            if(getIsSwitching()) {
                // Closing (canceling) any other extension threads to prevent race conditions and memory leaks
                for(int i = 0; i < runningThreads.size(); i++) {
                    final ConditionalThread thread = runningThreads.get(i);
                    if(thread.identifiers.getType() == ThreadIdentifiers.SWITCH) {
                        thread.close();
                        runningThreads.remove(thread);
                        i--;
                    }
                }
            }

            // Set initial conditions
            final boolean useRightTrigger = linearSlideState == LinearSlideStates.SPECIMEN_PLACE;
            final AutoArmRunner2.LinearSlideStates intialState = linearSlideState; 
            setIsSwitching(true);
            hasStartedSwitch = true;
            
            // Initialize the process for switiching
            final ConditionalThread buttonPresser = new ConditionalThread("switchToSpecimenGrab.buttonPresser", ThreadIdentifiers.SWITCH);
            final ConditionalThread armSwitcher = new ConditionalThread("switchToSpecimenGrab.armSwitcher", ThreadIdentifiers.SWITCH);

            buttonPresser.finishInitialization(
                () -> linearSlideState.equals(LinearSlideStates.SPECIMEN_RETRACT), 
                (Boolean unusedParam) -> {
                    if(useRightTrigger) {
                        gamepad2.right_trigger = 1.0f;
                    } else {
                        gamepad2.left_trigger = 1.0f;
                    }
                }, 
                (Boolean unusedParam) -> {
                    if(useRightTrigger) {
                        gamepad2.right_trigger = 0.0f;
                    } else {
                        gamepad2.left_trigger = 0.0f;
                    }
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
                    isPreloaded = false;
                } 
            );

            // Starting the processes
            isPreloaded = true;
            runningThreads.add(buttonPresser);
            runningThreads.add(armSwitcher);
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
            final ElapsedTime closeTimer = new ElapsedTime();
            closeTimer.reset();
            isOpen = false;
            for(int i = 0; i < runningThreads.size() && closeTimer.seconds() < SECONDS_CLOSE_TIMEOUT; i++) {
                final Closeable closeableThread = runningThreads.get(i);
                try {
                    closeableThread.close();
                } catch(IOException | ConcurrentModificationException err) {
                }
                // i--;
            }
            runningThreads.clear();
            try {
                interrupt();
            } catch(SecurityException err) {
                // Not much I want to do here
            }
        }

        /**
         * Makes the calling thread wait for the previous call of a method on 
         * this object to have finished waiting asynchronously.
         */
        public void waitForFinish() throws InterruptedException {
            while((getIsWaiting() && isOpen) && opModeIsActive()) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }
        }

        /**
         * Makes the calling thread wait for the previous call of switchArmMode 
         * to have finished executing the arm switch.
         */
        public void waitForSwitch() throws InterruptedException {
            while((getIsSwitching() && isOpen) && opModeIsActive()) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }    
        }
        
        /**
         * Makes the calling thread wait for the previous call of a extendSlides 
         * to have finished extending/retracting to the positioin.
         */
        public void waitForExtension() throws InterruptedException {
            while((getIsExtending() && isOpen) && opModeIsActive()) {
                Thread.sleep(30); // Wait and give breathing room to the other thread(s)
            }
        }

        /**
         * Makes the calling thread wait for the previous call of switchArmMode 
         * to have registered the button press and started the switch
         */
        public void waitForSwitchStart() throws InterruptedException {
            while((!getHasStartedSwitch() && isOpen) && opModeIsActive()) {
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
                while((currentBoolean = (!condition.getAsBoolean() && isOpen)) && opModeIsActive()) {
                    onContinue.accept(currentBoolean);
                    Thread.sleep(30); // Allow for the process in other threads to continue;
                }

                onFinish.accept(!condition.getAsBoolean());
            } catch(InterruptedException err) {
                // telemetry.addData("Interupted Running Conditional Thread: ", err.getMessage());
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

    protected final LiftHandlerThread lift = new LiftHandlerThread(); 
    private final ArrayList<ConditionalThread> quedThreads = new ArrayList<ConditionalThread>();
    private ElapsedTime autoRuntime = new ElapsedTime();
    final VelConstraint slowAtEnd = (robotPose, path, disp) -> {
        // Slow if closer to end
        final double MIN_SPEED = 20;
        final double MAX_SPEED = globalDrive.PARAMS.maxWheelVel;
        final double DECREASE_DIST = 5;
        return Math.min(
            Math.abs(END_X - robotPose.position.x.value()) 
                / DECREASE_DIST 
                * (MAX_SPEED - MIN_SPEED) 
                + MIN_SPEED, 
            MAX_SPEED
        );
    };

    @Override
    public void opMode_init() {
        super.opMode_init();

        isTelemetrySuppresed = false;
        
        // Initializing other hardware(-ish) bits
        globalDrive = new MecanumDrive(hardwareMap, START_LOCATION);
        sampleSensingDistance = hardwareMap
            .get(ColorRangeSensor.class, "sampleSensor")
            .getDistance(DistanceUnit.CM);

        // lift.start();
        initHardware();
        

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
        
            teleopModeToggle = new ButtonPressHandler(gamepad1, "back", (Gamepad g) -> {
                isTeleopMode = !isTeleopMode;
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
        while((Math.abs(getLinearSlideAvgPosition() - target) > tolerance) && opModeIsActive()) {
            // driveMotorToTo(linearSlideLift, target, tolerance, power);
            setFightingGravity(false);
            linearSlideLeft.setTargetPosition(target);
            linearSlideLeft.setTargetPositionTolerance(tolerance);
            linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(power);

            linearSlideRight.setTargetPosition(target);
            linearSlideRight.setTargetPositionTolerance(tolerance);
            linearSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearSlideRight.setPower(power);
        }

        setFightingGravity(true);
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
        lift.extendSlides(extendToSampleExtension, GRAB_EXTENSION_TOLERANCE, EXTENSION_POWER);
    }

    private void extendSync(int offset) {
        final int target = extendToSampleExtension + offset;
        final int tolerance = GRAB_EXTENSION_TOLERANCE;
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
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        lift.extendSlides(FULLY_RETRACTED, RETRACTION_TOLERANCE, RETRACTION_POWER);
    }

    private void retractSync() {
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        final int target = FULLY_RETRACTED;
        final int tolerance = RETRACTION_TOLERANCE;
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
        // linearSlideLeft.setPower(0);
        // linearSlideRight.setPower(0);
        intakeWheelR.setPower(INTAKE_POWER_MAX);
        intakeWheelL.setPower(INTAKE_POWER_MAX);
        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
    }

    private void grabSpecimenAsync() throws InterruptedException {
        specimenGrabberL.setPosition(SERVO_VALUES.specimenGrabberClosePos);
        specimenGrabberR.setPosition(SERVO_VALUES.specimenGrabberClosePos);
    }

    private void releaseSpecimenAsync() throws InterruptedException {
        specimenGrabberL.setPosition(SERVO_VALUES.specimenGrabberOpenPos);
        specimenGrabberR.setPosition(SERVO_VALUES.specimenGrabberOpenPos);
    }

    private void hoverSampleAsync() throws InterruptedException {
        intakePivot.setPosition(SERVO_VALUES.pivotHoverPos);
        intakeWheelR.setPower(INTAKE_POWER_MAX);
        intakeWheelL.setPower(INTAKE_POWER_MAX);
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
        if(DO_TURN_DEPO) {
            intakePivot.setPosition(SERVO_VALUES.pivotDepositPos);
        }
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
            BUCKETS_EXTENSION_TOLERANCE, 
            EXTENSION_POWER
        );
    }

    private void extendToBucketsSync() {
        final int target = (int) AutoArmRunner2.SLIDE_CONSTANTS.topBucketHeightAlternate;
        final int tolerance = BUCKETS_EXTENSION_TOLERANCE;
        final double speed = EXTENSION_POWER;
        driveLiftTo(target, tolerance, speed);
    }

    /**
     * Extends the arm to the buckets once the arm has sufficiently switched.
     * Both the waiting and the extending are done asynchronously. To wait for
     * the extension to finish while blocking the calling thread, use 
     * {@code LiftHandlerThread.waitForExtension()} (or {@code LiftHandlerThread
     * .waitForFinish()}).
     * 
     * @param extendBound How far up the pivot must be before extending, in 
     *     pivot ticks 
     */
    private void queExtendToChamberAsync(double extendBound) {
        // Make the thread that waits for the switch to finish
        final ConditionalThread queThread = new ConditionalThread("queExtendToChamberAsync.queThread", ThreadIdentifiers.QUE);
        queThread.finishInitialization(
            () -> getLinearPivotAvgPosition() >= extendBound, 
            (Boolean unusedParam) -> {
                extendToChambersAsync();
                quedThreads.remove(queThread);
            }
        );
        quedThreads.add(queThread);
        queThread.start();
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
        sleep(SPECIMEN_GRAB_MS); // To stop from accidentally moving with the sample still in the robot's maw
        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

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
     * Returns the number of seconds remaining in the auto opmode. This is since 
     * the press of the play button, NOT the init button. 
     * 
     * <p> If the auto timer is disabled (i.e. what stops the opmode after 30s),
     * this is able to go into the negatives 
     * 
     * @return What sure what else to say ¯\_(ツ)_/¯
     */
    private double getSecondsRemaining() {
        return 30 - autoRuntime.seconds();
    }

    /**
     * Extends the linear slides so that the specimen can hook
     */
    private void extendToChambersAsync() {
        lift.extendSlides(CHAMBER_EXTENSION, BUCKETS_EXTENSION_TOLERANCE, 1.0);
    }

    private void hookChamberSync() throws InterruptedException {
        driveLiftTo(HOOK_RETRACTION, RETRACTION_TOLERANCE, 0.5);
    }

    private void pivotDownSync() throws InterruptedException {
        gamepad2.right_trigger = 1.0f;
        
        // Wait for the input to be registered and the position completed
        while(
            !(
                Math.abs(getLinearPivotAvgPosition() - PIVOT_CONSTANTS.specimenPlacePos) <= 0.3 * PIVOT_TICKS_PER_DEGREE
                && linearSlideState.equals(LinearSlideStates.SPECIMEN_PLACE)
            )
            && opModeIsActive()
        ) {
            Thread.sleep(30);
        }

        gamepad2.right_trigger = 0.0f;
    }

    /**
     * Moves the robot to the chambers and places the specimen. The linear 
     * slides are in the slightly retracted position, and the pivot is in deposit.
     * The arm is NOT pivoting down after invoking this method.
     */
    private void moveAndPlaceSpecimenInitial(TranslationalVelConstraint cont, ProfileAccelConstraint cont2) 
        throws InterruptedException 
    {
        // lineTo( globalDrive, addPoses(getCurrentPosition(), new Pose2d(8, 0, 0)), cont, cont2 );
        lift.switchToChamber();
        queExtendToChamberAsync(SPECIMEN_SAFE_FOR_EXTENSION);

        // Extend, move, and wait for the switch before hooking
        final double SAFETY_DIST = 24; // Used to prevent contact with the 
        final double ALLIANCE_SHARING_DIST = 2; // Inches from the tile teeth, for space

        final Vector2d dest = new Vector2d(
            -48 + ROBOT_CENTER.position.x + SPECIMEN_PLACE_OFFSET, 
            -(BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y + ALLIANCE_SHARING_DIST)
        );
        resetDestinationOffset();

        globalDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        lineTo(globalDrive, dest, cont, cont2);
        globalDrive.PARAMS.positionTolerance = 1.0;
        
    
        // Wait for the pivot to be reasonably rotated before extending 
        while(
            getLinearPivotAvgPosition() < SPECIMEN_SAFE_FOR_EXTENSION
            && linearSlideState != LinearSlideStates.SPECIMEN_POSITION
            && opModeIsActive()
        ) {
            Thread.sleep(30); // Give time to other threads to do their thang
        }
        
        // extendToChambersAsync();
        lift.waitForFinish();

        // Moving forward and hooking onto the chamber'
        pivotDownSync(); // Lowering the motor to prevent collisions
        hookChamberSync();
        resetDestinationOffset();
    }

    /**
     * Moves the robot to the chambers and places the specimen. The linear 
     * slides are in the slightly retracted position, and the pivot is in deposit.
     * The arm is NOT pivoting down after invoking this method.
     */
    private void moveAndPlaceSpecimenRepeated(
        double lateralPlaceOffset,
        TranslationalVelConstraint cont, 
        ProfileAccelConstraint cont2
    ) throws InterruptedException {
        // lineTo( globalDrive, addPoses(getCurrentPosition(), new Pose2d(8, 0, 0)), cont, cont2 );
        lift.switchToChamber();
        queExtendToChamberAsync(SPECIMEN_SAFE_FOR_EXTENSION);

        // Extend, move, and wait for the switch before hooking
        final double SAFETY_DIST = 24; // Used to prevent contact with the 

        final Pose2d dest = new Pose2d(
            -48 + ROBOT_CENTER.position.x + SPECIMEN_PLACE_OFFSET, 
            -(BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y + ALLIANCE_SHARING_DIST_REPEATED) + lateralPlaceOffset,
            Math.toRadians(0)
        );
        resetDestinationOffset();

        globalDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(new Pose2d(-SAFE_PLACE_DIST, 0, 0));
        final TrajectoryActionBuilder lineToBuilder = getLineToLinearHeadingTrajectory(
            globalDrive, 
            dest, 
            cont, 
            cont2
        );
        resetDestinationOffset();

        Actions.runBlocking(
            lineToBuilder
                .setTangent(0)
                .lineToXSplineHeading(dest.position.x, dest.heading.toDouble())
                .build()
        );
        globalDrive.PARAMS.positionTolerance = 1.0;
        
    
        // Wait for the pivot to be reasonably rotated before extending 
        while(
            getLinearPivotAvgPosition() < SPECIMEN_SAFE_FOR_EXTENSION
            && linearSlideState != LinearSlideStates.SPECIMEN_POSITION
            && opModeIsActive()
        ) {
            Thread.sleep(30); // Give time to other threads to do their thang
        }

        
        // extendToChambersAsync();
        lift.waitForFinish();

        // Moving forward and hooking onto the chamber'
        pivotDownSync(); // Lowering the motor to prevent collisions
        hookChamberSync();
        resetDestinationOffset();
    }

    private void moveAndGrabSpecimen() throws InterruptedException {
        final Pose2d curPos = getCurrentPosition();

        // Raising the arm
        if(linearSlideState != LinearSlideStates.SPECIMEN_GRAB) {
            lift.switchToSpecimenGrab();
        }

        // Moving to the spot
        globalDrive.PARAMS.readjustmentTime = GRAB_PAUSE;
        globalDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        Actions.runBlocking(globalDrive.actionBuilder(curPos)
            .setTangent(Math.toRadians(-90))
            .splineToSplineHeading(
                new Pose2d(
                    END_X, 
                    -42
                    ,Math.toRadians(180)
                ),
                Math.toRadians(180)
            )
            .lineToX(-72 + ROBOT_CENTER.position.x, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15, 40))
            .build()
        );
        globalDrive.PARAMS.positionTolerance = 1.0;
        globalDrive.PARAMS.readjustmentTime = 1.0;

        lift.waitForFinish();

        // Extending and grabbing the specimen
        // extendToGrabSpecimenSync();
        grabSpecimenAsync();
        sleep(SPECIMEN_GRAB_MS);
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

    protected void main(boolean arg) throws InterruptedException {
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
        specimenGrabberL.setPosition(SERVO_VALUES.specimenGrabberClosePos);
        specimenGrabberR.setPosition(SERVO_VALUES.specimenGrabberClosePos);

        // Putting on the preloaded specimen
        moveAndPlaceSpecimenInitial(
            new TranslationalVelConstraint(MAX_NET_VEL), 
            new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL)
        );
        
        lift.switchToSpecimenGrab();


        // Grabbing the specimen from the wall
        for(int i = 0; i < GRAB_FIRST; i++) {
            // Grab from the wall
            moveAndGrabSpecimen();

            // Go and place the specimen 🐎
            moveAndPlaceSpecimenRepeated(
                (i + 1) * SPECIMEN_LATERAL_INCREMENT,
                new TranslationalVelConstraint(MAX_NET_VEL), 
                new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL)
            );
        }
    
        // Moving the samples
        TrajectoryActionBuilder moveSamples = globalDrive.actionBuilder(getCurrentPosition())
            .setTangent(Math.toRadians(-90));

        for(int i = 0; i < PUSH_ITER; i++) {
            final double spikeY = BLUE_COLORED_TAG.position.y + 24 - DIST_INCREMENT * i;
            double distAway = JUST_MISSED_IT_DIST;
            
            if(i == 0) {
                distAway = 5;
            }

            // Pushing in the sample from the mark
            moveSamples = moveSamples
                .splineToSplineHeading(
                    new Pose2d(
                        BLUE_SPIKE_X, 
                        spikeY + distAway + ROBOT_CENTER.position.y
                        ,Math.toRadians(0)
                    ), 
                    Math.toRadians(0)
                    
                    // new AngularVelConstraint(2 * Math.PI)
                )
                .splineToConstantHeading(
                    new Vector2d(
                    // new Pose2d(
                        BLUE_SPIKE_X + DIST_FROM, 
                        Math.max(-72, spikeY - CAPTURE_BREADTH) + ROBOT_CENTER.position.y
                        // ,Math.toRadians(90)
                    ),
                    Math.toRadians(180)
                    // new AngularVelConstraint(2 * Math.PI)
                )
                .lineToXConstantHeading(END_X, slowAtEnd)
                // .endTrajectory()
                
                // .setTangent(0)

                // The specimen has just been grabbed; go place it!
                // .setReversed(true)
                ;
        }

        // Push the colored samples
        Actions.runBlocking(new ParallelAction( 
            // Driving to the pickup point
            moveSamples.build()
        ));

        // Grabbing the specimen from the wall
        for(int i = GRAB_FIRST; i < GRAB_ITER; i++) {
            // Grab from the wall
            moveAndGrabSpecimen();

            // Go and place the specimen 🐎
            moveAndPlaceSpecimenRepeated(
                (i + 1) * SPECIMEN_LATERAL_INCREMENT,
                new TranslationalVelConstraint(MAX_NET_VEL), 
                new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL)
            );
        }

    }

    @Override
    public void opMode_start() {
        try {
            if(!isTeleopMode) { 
                main(isTeleopMode || isDebugMode);
                // main(true);
            } else {
                lift.start();
            }
        } catch(InterruptedException err) {
            telemetry.addData("!! CAUGHT FATAL ERROR IN MAIN", err.getMessage());
            telemetry.update();
            // telemetry.clear();
        }
    }

    private ButtonPressHandler buttonAHandler = null;
    private ButtonPressHandler buttonBHandler = null;
    private ButtonPressHandler buttonXHandler = null;
    private ButtonPressHandler buttonYHandler = null;
    private ButtonPressHandler buttonUPHandler = null;
    private ButtonPressHandler buttonDOWNHandler = null;
    private ButtonPressHandler buttonLEFTHandler = null;
    private ButtonPressHandler buttonRIGHTHandler = null;
    private ButtonPressHandler buttonLTHandler = null;
    private ButtonPressHandler buttonRTHandler = null;
    private ButtonPressHandler buttonLBHandler = null;
    private ButtonPressHandler buttonRBHandler = null;
    private ButtonPressHandler buttonLJHandler = null;
    private ButtonPressHandler buttonRJHandler = null;
    private ButtonPressHandler buttonSTARTHandler = null;

    @Override
    public void opMode_loop() {
        try {
            if(isTeleopMode) {
                teleopLoop();
            }
        } catch(Exception e) {
            telemetry.addData("!!!EXCEPTION!!!", e.toString());
            telemetry.update();
        }
    }

    private boolean a = false;
    private boolean b = false;
    private boolean x = false;
    private boolean y = false;
    private boolean dpad_up = false;
    private boolean dpad_down = false;
    private boolean dpad_left = false;
    private boolean dpad_right = false;
    private boolean left_trigger = false;
    private boolean right_trigger = false;
    private boolean left_bumper = false;
    private boolean right_bumper = false;
    private boolean left_stick_button = false;
    private boolean right_stick_button = false;
    private boolean start = false;

    private void teleopLoop() throws InterruptedException, IllegalAccessException, NoSuchFieldException {
        if(!isTeleopMode) {
            return;
        }

        // Look, Ma! It's an age structure diagram!
        a                  =                   a && gamepad1.a                   ;
        b                  =                   b && gamepad1.b                   ;
        x                  =                   x && gamepad1.x                   ;
        y                  =                   y && gamepad1.y                   ;
        dpad_up            =             dpad_up && gamepad1.dpad_up             ;
        dpad_down          =           dpad_down && gamepad1.dpad_down           ;
        dpad_left          =           dpad_left && gamepad1.dpad_left           ;
        dpad_right         =          dpad_right && gamepad1.dpad_right          ;
        left_trigger       =        left_trigger && gamepad1.left_trigger > 0.1  ;
        right_trigger      =       right_trigger && gamepad1.right_trigger > 0.1 ;    
        left_bumper        =         left_bumper && gamepad1.left_bumper         ;
        right_bumper       =        right_bumper && gamepad1.right_bumper        ;
        left_stick_button  =   left_stick_button && gamepad1.left_stick_button   ;
        right_stick_button =  right_stick_button && gamepad1.right_stick_button  ;
        start              =               start && gamepad1.start               ;
        
        
        
        
        // 
        if(gamepad1.a && !a) {switchArmAsync(); a =true;}

        if(gamepad1.b && !b) {extendAsync(); b = true;}
        
        if(gamepad1.x && !x) {extendSync(0); x = true;}
        
        if(gamepad1.y && !y) {retractAsync(); y = true;}
        
        if(gamepad1.dpad_up && !dpad_up) {retractSync(); dpad_up = true;}
        
        if(gamepad1.dpad_down && !dpad_down) {grabSampleAsync(); dpad_down = true;}
        
        if(gamepad1.dpad_left && !dpad_left) {depositAsync(); dpad_left = true;}
        
        if(gamepad1.dpad_right && !dpad_right) {extendToBucketsAsync(); dpad_right = true;}
        
        if(gamepad1.left_trigger > 0.1 && !left_trigger) {releaseSpecimenAsync(); left_trigger = true;}
        
        if(gamepad1.right_trigger > 0.1 && !right_trigger) {grabSpecimenAsync(); right_trigger = false;}
        
        if(gamepad1.left_bumper && !left_bumper) {pivotDownSync(); left_bumper = true;}
        
        if(gamepad1.right_bumper && !right_bumper) {lift.switchToSpecimenGrab(); right_bumper = true;}        
        
        if(gamepad1.start && !start) {lift.switchToChamber(); start = true;}

        // TELEMETRY
        isTelemetrySuppresed = true;
        telemetry.setAutoClear(true);
        telemetry.setMsTransmissionInterval(33);
        if(isDebugMode) {
            lift.print();

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addData("Current State", linearSlideState.name());
            telemetry.addLine("");

            telemetry.addLine("ACTUATORS");
            // telemetry.addData("LAR POW", linearActuatorRight.getPower());
            // telemetry.addData("LAL POW", linearActuatorLeft.getPower());
            telemetry.addLine("-------------------------");

            telemetry.addLine("DRIVETRAIN");
            telemetry.addData("Front R POW", frontRight.getPower());
            telemetry.addData("Back R POW", backLeft.getPower());
            telemetry.addData("Front R POW", frontRight.getPower());
            telemetry.addData("Back L POW", backLeft.getPower());
            telemetry.addLine("-------------------------");

            telemetry.addLine("LIFT SLIDES");
            telemetry.addData("Slide R POW", linearSlideRight.getPower());
            telemetry.addData("Slide L POW", linearSlideLeft.getPower());
            telemetry.addData("Slide R POS", linearSlideRight.getCurrentPosition());
            telemetry.addData("Slide L POS", linearSlideLeft.getCurrentPosition());
            telemetry.addData("Slide AVG POS", getLinearSlideAvgPosition());
            // telemetry.addData("Slide AVG Extension (in)", liftTicksToInches(linearSlideAvgPosition));
            telemetry.addLine("-------------------------");

            telemetry.addLine("LIFT PIVOTS");
            telemetry.addData("Pivot R POW", linearPivotRight.getPower());
            telemetry.addData("Pivot L POW", linearPivotLeft.getPower());
            telemetry.addData("Pivot R POS", linearPivotRight.getCurrentPosition());
            telemetry.addData("Pivot L POS", linearPivotLeft.getCurrentPosition());
            telemetry.addData("Pivot AVG POS", getLinearPivotAvgPosition());
            // telemetry.addData("Pivot AVG Extension (deg)", 360 / (2 * Math.PI) * pivotTicksToRadians(linearPivotAvgPosition));
            telemetry.addLine("-------------------------");

            telemetry.addLine("SERVOS");
            telemetry.addData("Intake WR POW", intakeWheelR.getPower());
            telemetry.addData("Intake WL POW", intakeWheelL.getPower());
            telemetry.addData("Intake PIVOT POS", intakePivot.getPosition());
            telemetry.addData("Specimen L POS", specimenGrabberL.getPosition());
            telemetry.addData("Specimen R POS", specimenGrabberR.getPosition());
            telemetry.addLine("-------------------------");

            telemetry.addLine("SENSORS");
            telemetry.addData("Limit Switch Activated", linearSlideSwitch.isPressed());
            telemetry.addData("Sample Sensor Gain", sampleSensor.getGain());
            telemetry.addData("Sample DIST (CM)", sampleSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine("(operating range 1-10 centimeters)");
            telemetry.addData("Red", sampleSensor.getNormalizedColors().red);
            telemetry.addData("Green", sampleSensor.getNormalizedColors().green);
            telemetry.addData("Blue", sampleSensor.getNormalizedColors().blue);
            telemetry.addLine("-------------------------");

            telemetry.addLine("LOGIC");
            telemetry.addData("Specimanning", specimanning);
            telemetry.addLine("-------------------------");
        }
    }

    @Override
    public void opMode_stop() {
        // powerDriveMotors(0, 0, 0, 0);
        lift.close();

        // Closing and removing for grabage collection every queing thread
        while(quedThreads.size() != 0) {
            quedThreads.get(0).close();
            quedThreads.remove(0);
        }
    }
}
