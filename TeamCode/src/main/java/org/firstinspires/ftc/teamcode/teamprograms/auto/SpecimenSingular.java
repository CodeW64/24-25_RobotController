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
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name="Single Specimen")
public class SpecimenSingular extends AutoCommonPaths {
    private boolean isBlue = true;
    private boolean shouldParkObservation = true; // False: Go to ascent zone there
    private int neutralTagId = AprilLocater.NEUTRAL_BLUE_ID;
    private int coloredTagId = AprilLocater.COLORED_BLUE_ID;
    private boolean repositionEnabled = false;
    // DEV: This line is true to do testing; please make it false by thurs
    private static boolean isTeleopMode = false; // FIXME: MAKE SURE THIS IS ____NEVER___ ENABLED OR ALLOWED IN A MATCH
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
        -24 + ROBOT_CENTER.position.y, //  0 is the top y-coord
        Math.toRadians(0)
    );

    private double sampleSensingDistance;

    public static double MAX_NET_VEL = 40;
    public static double MIN_NET_ACCEL = 50;
    public static double MAX_NET_ACCEL = 50;
    
    public static int DEPOSIT_EJECT_MS = 200;
    public static double SECONDS_CLOSE_TIMEOUT = 0.5;
    public static double GRAB_MIN_REMAINING_SEC = 4.0;
    public static double GRAB_SYNC_MAX_SEC = 5.0;
    public static double GRAB_RETRY_SEC = 1.5;
    public static double MIN_REAMINING_SCORE_SEC = 3.0;

    public static int CHAMBER_EXTENSION = 1300;
    public static int EXTEND_TO_SAMPLE_EXTENSION = 1150;
    public static int EXTEND_TO_SAMPLE_LAST_EXTENSION = 1300;
    public static int FULLY_RETRACTED = 500;
    public static int GRAB_EXTENSION_TOLERANCE = 30;
    public static int RETRACTION_TOLERANCE = 40;
    public static int BUCKETS_EXTENSION_TOLERANCE = 10;
    
    public static double NET_ZONE_TOLERANCE = 0.7;

    public static double SQRT2 = Math.sqrt(2); // approx 1.4142
    public static double DIST_BACK = 8.1;
    public static double DIST_BACK_LATER = 8.8;
    public static double DIST_INCREMENT = -0.5; // NOTE: change this when roadRunner is tuned
    public static double DIST_STRAFE = 2.5;
    public static double DIST_STRAFE_LATER = 2.5;
    public static double SPECIMEN_PLACE_OFFSET = 1.0;
    public static boolean DO_TURN_DEPO = false; // Should the robot turn the pivot when depositing?
    
    // TODO: These are not able to be edited by Dash. Make them constructed when needed?
    //       Another thought: Make a class "PoseInit" with x, y, and theta attributes,
    //       and construct the Pose2ds at runtime? Call a method on the PoseInits to convert
    //       to a Pose2d? Food for thought. 🍗
    public static Pose2d RETRY_OFFSET = new Pose2d(0.5, 0.5, 0);
    public static Pose2d INTAKE_OFFSET = new Pose2d(-6.25, 0, -Math.PI / 2);
    public static Vector2d GRABBING_DISTANCE = new Vector2d(-EXTEND_TO_SAMPLE_EXTENSION / LIFT_TICKS_PER_INCH_EXTENDED, 0);
    public static Pose2d NORMAL_GRAB_OFFSET = new Pose2d(1, -1, 0);
    public static double LAST_SPIKE_ANGLE = Math.toRadians(90);

    public static int MAX_GRAB_INDEX = -1; // DEV: Make this so that the loop can actually run
    public static int MIN_GRAB_INDEX = Integer.MAX_VALUE; // DEV: Make this so that the loop can actually run
    
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

            if(USE_PIVOT_TOLERANCE && linearSlideState.equals(LinearSlideStates.SPECIMEN_GRAB)) {
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
            final ConditionalThread buttonPresser = new ConditionalThread("switchArmMode.buttonPresser", ThreadIdentifiers.SWITCH);
            final ConditionalThread armSwitcher = new ConditionalThread("switchArmMode.armSwitcher", ThreadIdentifiers.SWITCH);

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
            interrupt();
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
        lift.extendSlides(FULLY_RETRACTED, RETRACTION_TOLERANCE, 0.5);
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
     */
    private void moveAndPlaceSpecimen(TranslationalVelConstraint cont, ProfileAccelConstraint cont2) 
        throws InterruptedException 
    {
        lineTo( globalDrive, addPoses(getCurrentPosition(), new Pose2d(8, 0, 0)), cont, cont2 );
        lift.switchToChamber();
    
        // Wait for the pivot to be reasonably rotated before extending 
        while(
            getLinearPivotAvgPosition() < SPECIMEN_SAFE_FOR_EXTENSION
            && linearSlideState != LinearSlideStates.SPECIMEN_POSITION
            && opModeIsActive()
        ) {
            Thread.sleep(30); // Give time to other threads to do their thang
        }

        // Extend, move, and wait for the switch before hooking
        final double SAFETY_DIST = 24; // Used to prevent contact with the 
        final double ALLIANCE_SHARING_DIST = 2; // Inches from the tile teeth, for space
        extendToChambersAsync();

        final Vector2d dest = new Vector2d(
            -48 + ROBOT_CENTER.position.x + SPECIMEN_PLACE_OFFSET, 
            -(BLUE_CHAMBER.position.y + ROBOT_CENTER.position.y + ALLIANCE_SHARING_DIST)
        );
        resetDestinationOffset();
        lineTo(globalDrive, dest, cont, cont2);
        lift.waitForFinish();

        // Moving forward and hooking onto the chamber'
        pivotDownSync(); // Lowering the motor to prevent collisions
        hookChamberSync();
        resetDestinationOffset();
    }

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

        // Driving to the chamber and scoring
        globalDrive.updatePoseEstimate();
        moveAndPlaceSpecimen(
            new TranslationalVelConstraint(MAX_NET_VEL), 
            new ProfileAccelConstraint(-MIN_NET_ACCEL, MAX_NET_ACCEL)
        );
        retractSync();
        switchArmAsync(); // Lowring the arm
        lift.waitForFinish();

        // Parking for 3 points
        lineTo(globalDrive, new Vector2d(-60, -60));
    }

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
        // powerDriveMotors(0, 0, 0, 0);
        lift.close();

        // Closing and removing for grabage collection every queing thread
        while(quedThreads.size() != 0) {
            quedThreads.get(0).close();
            quedThreads.remove(0);
        }
    }
}
