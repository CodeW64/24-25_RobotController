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
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.AngularVelConstraint;
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
@Autonomous(name="Basket Placer (Preloaded Sample 2)")
public class SamplePreloadBasket2 extends AutoCommonPaths {
    public static boolean IS_BLUE = true;
    private boolean isBlue = IS_BLUE;
    private boolean shouldParkObservation = true; // False: Go to ascent zone there
    private int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    private int coloredTagId = AprilLocater.COLORED_BLUE_ID;
    private boolean repositionEnabled = false;
    // DEV: This line is true to do testing; please make it false by thurs
    public static boolean isTeleopMode = false; // FIXME: MAKE SURE THIS IS ____NEVER___ ENABLED OR ALLOWED IN A MATCH
    private static boolean isDebugMode = false;

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
    private final Pose2d ROBOT_INIT_POSE = new Pose2d(16.5, 17, Math.toRadians(0));
    
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
         48 - ROBOT_CENTER.position.y, //  0 is the top y-coord
        Math.toRadians(0)
    );

    private double sampleSensingDistance;

    public static double MAX_NET_VEL = 40;
    public static double MIN_NET_ACCEL = 50;
    public static double MAX_NET_ACCEL = 50;

    public static double SEARCH_TURN = Math.toRadians(-1);
    public static double MAX_SEARCH_ANG_VEL = Math.toRadians(1);
    
    public static int DEPOSIT_EJECT_MS = 200;
    public static double SECONDS_CLOSE_TIMEOUT = 0.5;
    public static double GRAB_MIN_REMAINING_SEC = 0;
    public static double GRAB_SYNC_MAX_SEC = 5.0;
    public static double GRAB_RETRY_SEC = 1.5;
    public static double MIN_REAMINING_SCORE_SEC = 3.0;

    public static int RETRACT_BACK = 30;
    public static int CHAMBER_EXTENSION = 1300;
    public static int EXTEND_INTO_SUBMERSIBLE = 1000;
    public static int SUBMERSIBLE_SEARCH_EXTENSION = 1500;
    public static int EXTEND_TO_SAMPLE_EXTENSION = 1500;
    public static int EXTEND_TO_SAMPLE_LAST_EXTENSION = 1300;
    public static int FULLY_RETRACTED = 500;
    public static int HOOK_RETRACTION = 300;
    public static int GRAB_EXTENSION_TOLERANCE = 30;
    public static int RETRACTION_TOLERANCE = 40;
    public static int BUCKETS_EXTENSION_TOLERANCE = 10;
    
    public static double NET_ZONE_TOLERANCE = 0.7;

    public static double SQRT2 = Math.sqrt(2); // approx 1.4142
    public static double DIST_BACK = 9.2;
    public static double DIST_BACK_LATER = 9.5;
    public static double DIST_INCREMENT = -1.0; // NOTE: change this when roadRunner is tuned
    public static double DIST_STRAFE = 2.5;
    public static double DIST_STRAFE_LATER = 2.5;
    public static double ALLIANCE_SHARING_DIST = 0; // Inches from the tile teeth, for space
    public static double SPECIMEN_PLACE_OFFSET = 0;
    // public static double SUBMERSIBLE_SEARCH_DIST = 3;
    public static double FINAL_BACK_AWAY = 8;
    
    
    // TODO: These are not able to be edited by Dash. Make them constructed when needed?
    //       Another thought: Make a class "PoseInit" with x, y, and theta attributes,
    //       and construct the Pose2ds at runtime? Call a method on the PoseInits to convert
    //       to a Pose2d? Food for thought. 🍗
    public static Pose2d RETRY_OFFSET = new Pose2d(1.0, 0.5, 0);
    public static Pose2d SUB_RETRY_OFFSET = new Pose2d(-0.5, -0.5, 0);
    public static Pose2d INTAKE_OFFSET = new Pose2d(-6.25, 0, -Math.PI / 2);
    public static Vector2d GRABBING_DISTANCE = new Vector2d(-EXTEND_TO_SAMPLE_EXTENSION / LIFT_TICKS_PER_INCH_EXTENDED, 0);
    public static Pose2d NORMAL_GRAB_OFFSET = new Pose2d(1, 0.85, 0);
    public static double LAST_SPIKE_ANGLE = Math.toRadians(90);

    public static int MAX_GRAB_INDEX = 2;
    public static int MIN_GRAB_INDEX = 1;
    
    public static double PIVOT_SAFE_FOR_EXTENSION = 60 * PIVOT_TICKS_PER_DEGREE; // Pivot pos when extension to buckets may occur
    public static double SPECIMEN_SAFE_FOR_EXTENSION = 1100; // Pivot pos when extension to buckets may occur
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

    public static double EXTENSION_POWER = 0.8; // Previously 0.15
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
    private class LiftHandlerThread extends CloseableThread {
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
                () -> linearSlideState.equals(LinearSlideStates.PIVOT_TO_SPECIMEN), 
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

    private final LiftHandlerThread lift = new LiftHandlerThread(); 
    private final ArrayList<ConditionalThread> quedThreads = new ArrayList<ConditionalThread>();
    private double distBack = DIST_BACK;
    private double distStrafe = DIST_STRAFE;
    private Pose2d backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0);
    private ElapsedTime autoRuntime = new ElapsedTime();

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

    private boolean driveLiftToIterative(int target, int tolerance, double power) {
        if((Math.abs(getLinearSlideAvgPosition() - target) > tolerance) && opModeIsActive()) {
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
            return true;
        } else {
            setFightingGravity(true);
            linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            linearSlideLeft.setPower(0);
            linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            linearSlideRight.setPower(0);
            return false;
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
        // Presssing the grab button
        linearSlideLeft.setPower(0);
        linearSlideRight.setPower(0);
        intakeWheelR.setPower(INTAKE_POWER_MAX);
        intakeWheelL.setPower(INTAKE_POWER_MAX);
        intakePivot.setPosition(SERVO_VALUES.pivotIntakePos);
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

    private void extendToSubmersibleAsync() {
        lift.extendSlides(
            (int) EXTEND_TO_SAMPLE_EXTENSION, 
            GRAB_EXTENSION_TOLERANCE, 
            EXTENSION_POWER
        );
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
    private void queExtendToBucketsAsync(double extendBound) {
        // Make the thread that waits for the switch to finish
        final ConditionalThread queThread = new ConditionalThread("queExtendToBucketsAsync.queThread", ThreadIdentifiers.QUE);
        queThread.finishInitialization(
            () -> getLinearPivotAvgPosition() >= extendBound, 
            (Boolean unusedParam) -> {
                extendToBucketsAsync();
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
    private void berriddenOfSample(boolean doArmSwitch) throws InterruptedException {
        // The arm is raised, so put it into the basket!
        depositAsync();
        sleep(DEPOSIT_EJECT_MS); // To stop from accidentally moving with the sample still in the robot's maw
        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);

        // Lowering and switching
        if(doArmSwitch) { 
            switchArmAsync();
        }
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
     * Extends the arm and grabs the sample. The distance is fixed. The arm 
     * attempts the grab only once. The arm remains extended afterwards.
     * 
     * <p> Estimated blocking runtime: 2750 ms
     * 
     * @return Whether or not the sample was obtained. 
     * @throws InterruptedException
     */
    private boolean grabSampleSequence(boolean retry, int offset) throws InterruptedException {
        telemetry.addLine("Extending to sample...");
        telemetry.update();
        lift.waitForFinish(); // Wait for the arm to finish retraction
        extendSync(offset);
        // lift.waitForExtension(); // Waiting for full extension

        final double grabSequenceStart = autoRuntime.seconds();
        retryLoop:
        while(
            (
                !isPossessingSample(currentSampleDistance) 
                && autoRuntime.seconds() - grabSequenceStart <= GRAB_SYNC_MAX_SEC
                && getSecondsRemaining() >= GRAB_MIN_REMAINING_SEC
            )
            && opModeIsActive()
        ) {
            final double retryStart = autoRuntime.seconds();
            telemetry.addLine("Switching to intake mode to grab...");
            telemetry.update();
            isStateInitialized = false;
            linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;
            grabSampleAsync(); // Grab the sample

            // Wait for completion
            telemetry.addLine("Sleeping");
            telemetry.update();
            // CAPUT MEUM DOLET.
            while(
                (
                    !linearSlideState.equals(LinearSlideStates.INTAKE_FULL) 
                    && !linearSlideState.equals(LinearSlideStates.INTAKE_EMPTY)
                    && autoRuntime.seconds() - retryStart <= GRAB_RETRY_SEC
                    && autoRuntime.seconds() - grabSequenceStart <= GRAB_SYNC_MAX_SEC
                    && getSecondsRemaining() >= GRAB_MIN_REMAINING_SEC
                    && !isPossessingSample(currentSampleDistance)
                )
                && opModeIsActive()
            ) {
                sleep(30); // Waiting whilst freeing CPU for other threads
            }

            // Exiting the retry loop if we don't want to retry
            if(!retry || isPossessingSample(currentSampleDistance) || getSecondsRemaining() < GRAB_MIN_REMAINING_SEC) {
                break retryLoop;
            }

            // Moving so that we have more chance of getting it.
            hoverSampleAsync();
            setDestinationOffset(RETRY_OFFSET); // Move from current position forward and a sample up
            lineTo(globalDrive, getCurrentPosition());
            resetDestinationOffset();
        }

        return isPossessingSample(currentSampleDistance);
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
     * Performs all necessary actions to take a grabbed sample to the basket. 
     * This includes moving to the basket, extending the arm, depositing, and a 
     * final retract.
     * 
     * <p> With the exception of an ending arm switch down, this method is 
     * synchronous. 
     */
    private void scoreSequence(int i) throws InterruptedException {
        // Getting the arm up and extended (at least partially) first, and the robot in a safe pos
        globalDrive.updatePoseEstimate();
        switchArmAsync(); // Get the arm up
        lift.waitForSwitchStart();

        queExtendToBucketsAsync(PIVOT_SAFE_FOR_EXTENSION);

        distBack = 2 * (DIST_BACK_LATER - DIST_INCREMENT * (2 - i));
        distStrafe = 2 * DIST_STRAFE_LATER;
        backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0); // don't go too close to the buckets
        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(backAway); // Move back to avoid accidental hanging
        moveRobotToNetSafety(isBlue, new TranslationalVelConstraint(MAX_NET_VEL), new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;

        // Moving to the basket and scoring when the arm is sufficiently up
        lift.waitForSwitch();
        
        distBack = 2 * (DIST_BACK_LATER - DIST_INCREMENT * (2 - i));
        distStrafe = 2 * DIST_STRAFE_LATER;
        backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0); // don't go too close to the buckets
        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        MecanumDrive.PARAMS.headingTolerance = Math.toRadians(1);
        setDestinationOffset(backAway); // Move back 4 inches to avoid accidental hanging
        moveRobotToNetZone(isBlue, new TranslationalVelConstraint(MAX_NET_VEL), new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;
        MecanumDrive.PARAMS.headingTolerance = Math.toRadians(5);

        // Waiting for the arm to be sufficiently extended before depositing
        // Arm is told to lift when teh arm is told to switch up (before the safety move)
        lift.waitForFinish();
        berriddenOfSample(i != MIN_GRAB_INDEX); // Aaaaand deposit, then come back!
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
     * Moves the robot to the chambers and places the specimen. The linear 
     * slides are in the slightly retracted position, and the pivot is in deposit.
     * The arm is NOT pivoting down after invoking this method.
     */
    private void moveAndPlaceSpecimen(TranslationalVelConstraint cont, ProfileAccelConstraint cont2) 
        throws InterruptedException 
    {
        // lineTo( globalDrive, addPoses(getCurrentPosition(), new Pose2d(8, 0, 0)), cont, cont2 );
        lift.switchToChamber();
        queExtendToChamberAsync(SPECIMEN_SAFE_FOR_EXTENSION);

        // Extend, move, and wait for the switch before hooking
        final double SAFETY_DIST = 24; // Used to prevent contact with the 
        final Vector2d dest = new Vector2d(
            -48 + ROBOT_CENTER.position.x + SPECIMEN_PLACE_OFFSET, 
            BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y + ALLIANCE_SHARING_DIST
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

    private class SearchForSampleAction implements Action {
        private boolean hasFoundValidSample = false;
        private Alliance coloredAlliance = Alliance.NEUTRAL;

        public SearchForSampleAction(Alliance coloredAlliance) {
            super();
            this.coloredAlliance = coloredAlliance;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            // Is there a valid sample?
            final Alliance currentAlliance = getCurrentSampleAlliance();
            final boolean isDesired = 
                   currentAlliance == coloredAlliance 
                || currentAlliance == Alliance.NEUTRAL;

            if(isDesired) {
                // A valid sample was found! Tell the program to stop moving and grab
                hasFoundValidSample = true;
                return false;
            }

            // No sample has been found; continue searching
            return true;
        }

        public boolean hasFoundValidSample() {
            return this.hasFoundValidSample;
        }
    }

    private void searchForSubmersibleSample(boolean findBlue) throws InterruptedException {
        // Getting the allowed alliance
        final Alliance coloredAlliance = findBlue ? Alliance.BLUE : Alliance.RED;

        // Get the arm down and into the submerisble
        // switchArmAsync();
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        driveLiftTo(EXTEND_INTO_SUBMERSIBLE, BUCKETS_EXTENSION_TOLERANCE, EXTENSION_POWER);
        intakePivot.setPosition(SERVO_VALUES.pivotRestPos);
        // extendToSubmersibleAsync();
        // lift.waitForFinish();

        // Running the action to search for the sample This moves the robot while 
        // searching. Once either the movement ends or the sample is found, we 
        // the action ends
        final Pose2d curPos = getCurrentPosition();
        // final Action moveInSubmersible = globalDrive.actionBuilder(curPos)
        //     .setTangent(Math.toRadians(90)) // Make the line vertical
        //     .lineToY(curPos.position.y + SUBMERSIBLE_SEARCH_DIST)
        //     .build();

        final Action rotateBot = globalDrive.actionBuilder(curPos)
            .turnTo(SEARCH_TURN, new TurnConstraints(MAX_SEARCH_ANG_VEL, -globalDrive.PARAMS.maxAngAccel, globalDrive.PARAMS.maxAngAccel))
            .build();

        final Action extendArmInSubmersible = new Action() {
            @Override
            public boolean run(TelemetryPacket p) {
                return driveLiftToIterative(SUBMERSIBLE_SEARCH_EXTENSION, GRAB_EXTENSION_TOLERANCE, EXTENSION_POWER);
            }
        };

        final SearchForSampleAction searchForSample = new SearchForSampleAction(coloredAlliance);
        
        // Actions.runBlocking(new RaceAction(moveInSubmersible, searchForSample));
        Actions.runBlocking(new RaceAction(extendArmInSubmersible, searchForSample));

        // Grabbing the sample if one was found
        if(searchForSample.hasFoundValidSample()) {
            EXTENSION_POWER *= 0.8;
            driveLiftTo((int) getLinearSlideAvgPosition() - RETRACT_BACK, 5, RETRACTION_POWER);// It misses a little; just go back
            grabSampleAsync();
            isStateInitialized = false;
            linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;

            // throw new RuntimeException("stuffs");

            // Waiting for the sample to be grasped or too much time elapses
            retryLoop:
            while(
                !(
                    linearSlideState.equals(LinearSlideStates.INTAKE_FULL)
                    || linearSlideState.equals(LinearSlideStates.INTAKE_EMPTY)
                )
                && opModeIsActive()
            ) {
                final double retryStart = autoRuntime.seconds();
                telemetry.addLine("Switching to intake mode to grab...");
                telemetry.update();
                grabSampleAsync(); // Grab the sample
    
                // Wait for completion
                telemetry.addLine("Sleeping");
                telemetry.update();
                // CAPUT MEUM DOLET.
                while(
                    (
                        !linearSlideState.equals(LinearSlideStates.INTAKE_FULL) 
                        && !linearSlideState.equals(LinearSlideStates.INTAKE_EMPTY)
                        && autoRuntime.seconds() - retryStart <= GRAB_RETRY_SEC
                        && getSecondsRemaining() >= GRAB_MIN_REMAINING_SEC
                        && !isPossessingSample(currentSampleDistance)
                    )
                    && opModeIsActive()
                ) {
                    sleep(30); // Waiting whilst freeing CPU for other threads
                }
    
                // Exiting the retry loop if we don't want to retry
                if(isPossessingSample(currentSampleDistance) || getSecondsRemaining() < GRAB_MIN_REMAINING_SEC) {
                    break retryLoop;
                }
    
                // Moving so that we have more chance of getting it.
                hoverSampleAsync();
                setDestinationOffset(SUB_RETRY_OFFSET); // Move from current position forward and a sample up
                lineTo(globalDrive, getCurrentPosition());
                resetDestinationOffset();
            }
            EXTENSION_POWER /= 0.8;
        }

        // Retracting the arm to avoid hitting 
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        driveLiftTo(Math.max(HOOK_RETRACTION - 100, 0), RETRACTION_TOLERANCE, RETRACTION_POWER);
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
        // ElapsedTime timer;
        // DEV END

        // Starting the actual stuffs
        lift.start();
        SERVO_VALUES.specimenGrabberOpenPos = 0.58;
        specimenGrabberL.setPosition(SERVO_VALUES.specimenGrabberClosePos);
        specimenGrabberR.setPosition(SERVO_VALUES.specimenGrabberClosePos);
        
        // Putting on the preloaded specimen
        scoreSequence(8);

        // Going into the submersible
        // switchArmAsync(); 
        // lift.waitForFinish();
        // searchForSubmersibleSample(isBlue);
        resetDestinationOffset();

        // Driving to the spike marks
        SERVO_VALUES.specimenGrabberOpenPos = 0.55;
        boolean isFirstSpikeSample = true;
        grabDepositLoop:
        for(int i = MAX_GRAB_INDEX; i >= MIN_GRAB_INDEX && opModeIsActive(); i--) {
            // Initial positioning data
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final double extraRotation = i == 0 ? Math.toRadians(20) : 0; // Rotate more cuz' last one's hard to get to. 
            final Pose2d intakeOffset = new Pose2d(-6.25, 0, extraRotation - Math.PI / 2); // Offset from bot center
            final Pose2d grabbingDistance = i == 0 
                ? new Pose2d(-EXTEND_TO_SAMPLE_EXTENSION / LIFT_TICKS_PER_INCH_EXTENDED, 0, 0) 
                : new Pose2d(-20.0, 0, 0);
            
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
            Pose2d finalOffset = new Pose2d(rotatedPosition, totalOffset.heading); 

            if(i == 0) {
                // finalOffset = addPoses(finalOffset, new Pose2d(-2, 5, 0));
                extendToSampleExtension = EXTEND_TO_SAMPLE_LAST_EXTENSION;
            } else {
                finalOffset = addPoses(finalOffset, new Pose2d(2, -0.25, 0));
            }
            
            // Moving to grab the sample
            setDestinationOffset(finalOffset); // Puts the robot into grabbing position
            globalDrive.updatePoseEstimate();
            setRotationType(RotationType.SPLINE);

            if(!isFirstSpikeSample) {
                extendAsync();
            }

            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            if(isFirstSpikeSample) {
                extendSync(0);
            }

            // Grabbing the pixel
            resetDestinationOffset();
            grabSampleSequence(true, 0); // Grab the pixel. and retract

            if(!isPossessingSample(currentSampleDistance)) {
                continue grabDepositLoop;
            }

            retractSync();

            // Waiting for the arm to go up and out of the way
            if(!isPossessingSample(currentSampleDistance)) {
                continue grabDepositLoop;
            }

            switchArmAsync(); // Switch the arm up
            
            // Moving to score
            distBack = DIST_BACK_LATER - DIST_INCREMENT * (2 - i);
            distStrafe = DIST_STRAFE_LATER;
            backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0); // don't go too close to the buckets
            setDestinationOffset(backAway); // Move back to avoid accidental hanging
            
            MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
            globalDrive.updatePoseEstimate();
            moveRobotToNetZoneCcw(isBlue, new Action() {
                @Override
                public boolean run(TelemetryPacket p) {
                    if(lift.hasFinishedPivot()) {
                        extendToBucketsAsync(); // Extend to the buckets once we have done our signature turn.
                        return false;
                    }
                    return true;
                } 
            }, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-30, 30));
            MecanumDrive.PARAMS.positionTolerance = 1.0;    
            
            resetDestinationOffset();
            lift.waitForFinish();

            berriddenOfSample(i != MIN_GRAB_INDEX);
            isFirstSpikeSample = false;
        }

        // Moving back to prevent hitting the arm on the buckets
        lineTo(
            globalDrive, 
            addPoses(getCurrentPosition(), new Pose2d(FINAL_BACK_AWAY, -FINAL_BACK_AWAY, 0)), // Go back a bit
            new TranslationalVelConstraint(MAX_NET_VEL), 
            null
        );

        // Retracting the arm and putting the intake in a good spot
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        lift.extendSlides(0, 30, -1.0);

        if(lift.isDepositPosition(linearSlideState)) {
            // Only switching __down__
            lift.switchArmMode();
        }
        lift.waitForFinish();

        // if(goToAscent) {
            // setDestinationOffset(new Pose2d(0, 24, 0));
            // moveRobotToAscent(); 
        // }

        lift.waitForFinish();

        // lift.close();

        telemetry.clearAll();
        telemetry.addLine(accumulated);
        telemetry.update();

        // Putting the arm at position zero for the teleop folks
        gamepad1.dpad_left = true;
        gamepad2.dpad_left = true;

        // Yipee! Finishing up
        telemetry.addData("Status", "Completed! 🥳");
        telemetry.update();
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

        if(gamepad1.b && !b) {extendAsync(); b = true;} // Perferct! (20inches)
        
        if(gamepad1.x && !x) {extendSync(0); x = true;} // Perfect!!! (20inches)
        
        if(gamepad1.y && !y) {retractAsync(); y = true;} // perfect?
        
        if(gamepad1.dpad_up && !dpad_up) {retractSync(); dpad_up = true;} // Worked but left an infinite loop one time
        
        if(gamepad1.dpad_down && !dpad_down) {grabSampleAsync(); dpad_down = true;}
        
        if(gamepad1.dpad_left && !dpad_left) {depositAsync(); dpad_left = true;} // Stayed in deposit state constantly; functioned otherwise
        
        if(gamepad1.dpad_right && !dpad_right) {extendToBucketsAsync(); dpad_right = true;} // Got to position but failed to hold (fell at power 0); see also memory leak
        
        if(gamepad1.left_trigger > 0.1 && !left_trigger) {extendToBucketsSync(); left_trigger = true;}
        
        if(gamepad1.right_trigger > 0.1 && !right_trigger) {grabSampleSequence(true, 0); right_trigger = false;} //
        
        // if(gamepad1.left_bumper && !left_bumper) {berriddenOfSample(); left_bumper = true;} // Worked, I guess?
        
        if(gamepad1.right_bumper && !right_bumper) {switchArmAsync(); right_bumper = true;} // Worked going into deps; never exited from it UNLESS you depositted        
        
        if(gamepad1.start && !start) {lift.switchToChamber(); start = true;}
        
        // if(buttonHandler) {moveAndPlaceSpecimen();} 

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
