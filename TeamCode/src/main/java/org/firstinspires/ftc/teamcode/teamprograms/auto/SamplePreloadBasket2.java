package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;
import org.firstinspires.ftc.teamcode.teamprograms.teleop.IntoTheDeepTeleop;

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
@Autonomous(name="Basket Placer 2 (Preloaded Sample)")
public class SamplePreloadBasket2 extends AutoCommonPaths {
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
        -72 + ROBOT_CENTER.position.y, // -72 is the left grid x-coord 
         48 - ROBOT_CENTER.position.x, //  0 is the top y-coord
         0  + ROBOT_CENTER.heading.toDouble() // 0 is the deafult rotation
    );

    private double sampleSensingDistance;
    
    public static double SECONDS_CLOSE_TIMEOUT = 0.5;
    public static double GRAB_SAMPLE_SYNC_MAX_SECONDS = 3.3;
    public static double ONE_RETRY_MAX_SECONDS = 1.1;

    public static int CHAMBER_EXTENSION = 2500;
    public static int EXTEND_TO_SAMPLE_EXTENSION = 1150;
    public static int EXTEND_TO_SAMPLE_LAST_EXTENSION = 1300;
    public static int FULLY_RETRACTED = 500;
    
    public static double NET_ZONE_TOLERANCE = 0.7;

    public static double SQRT2 = Math.sqrt(2); // approx 1.4142
    public static double DIST_BACK = 8.1;
    public static double DIST_BACK_LATER = 8.8;
    public static double DIST_INCREMENT = -0.5; // NOTE: change this when roadRunner is tuned
    public static double DIST_STRAFE = 2.5;
    public static double DIST_STRAFE_LATER = 2.5;
    public static boolean DO_TURN_DEPO = false; // Should the robot turn the pivot when depositing?
    
    public final Pose2d retryOffset = new Pose2d(0.5, 0.5, 0);
    
    public static double PIVOT_SAFE_FOR_EXTENSION = 60 * PIVOT_TICKS_PER_DEGREE; // Pivot pos when extension to buckets may occur
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
                    linearSlideLeft.setPower(0);
                    linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
            if(USE_PIVOT_TOLERANCE && linearSlideState.equals(AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE)) {
                pivotTolerance = getPivotTolerance();
            }

            return 
                Math.abs(getLinearPivotAvgPosition() - linearPivotTargetPosition) <= pivotTolerance
                && (
                    linearSlideState.equals(AutoArmRunner2.LinearSlideStates.DEPOSIT_ACTIVE)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_ACTIVE)
                    || linearSlideState.equals(AutoArmRunner2.LinearSlideStates.INTAKE_FULL)
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
            // Set initial conditions
            setIsSwitching(true);
            hasStartedSwitch = false;
            
            // Initialize the process for switiching
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

    @Override
    public void opMode_init() {
        super.opMode_init();
        
        // telemetry.addData("Real IMU heading (DEG)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.DEGREES));
        // telemetry.addData("Real IMU heading (RAD)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.RADIANS));
        // telemetry.addLine();
        // telemetry.addData("Real IMU heading Vel (DEG)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.DEGREES));
        // telemetry.addData("Real IMU heading Vel (RAD)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.RADIANS));
        // telemetry.update();
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
        lift.extendSlides(extendToSampleExtension, 10, EXTENSION_POWER);
    }

    private void extendSync(int offset) {
        final int target = extendToSampleExtension + offset;
        final int tolerance = 10;
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
        lift.extendSlides(FULLY_RETRACTED, 30, RETRACTION_POWER);
    }

    private void retractSync() {
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
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
        intakeWheelR.setPower(INTAKE_POWER_MAX);
        intakeWheelL.setPower(INTAKE_POWER_MAX);
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
    private void berriddenOfSample() throws InterruptedException {
        // The arm is raised, so put it into the basket!
        depositAsync();
        sleep(200); // To stop from accidentally moving with the sample still in the robot's maw
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
     * Extends the arm and grabs the sample. The distance is fixed. The arm 
     * attempts the grab only once. The arm remains extended afterwards.
     * 
     * <p> Estimated blocking runtime: 2750 ms
     * 
     * @return Whether or not the sample was obtained. 
     * @throws InterruptedException
     */
    private boolean grabSampleSync(boolean retry, int offset) throws InterruptedException {
        telemetry.addLine("Extending to sample...");
        telemetry.update();
        lift.waitForFinish(); // Wait for the arm to finish retraction
        extendSync(offset);
        // lift.waitForExtension(); // Waiting for full extension

        isStateInitialized = false;
        linearSlideState = LinearSlideStates.INTAKE_ATTEMPT_SAMPLE;

        retryLoop:
        while(
            (
                !isPossessingSample(currentSampleDistance) 
                && lightTimer.seconds() <= 4.0 
                // && 30 - getRuntime() >= 5 // FIXME: Make this acutally representative of time left
            )
            && opModeIsActive()
        ) {
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
                    && !linearSlideState.equals(LinearSlideStates.INTAKE_ACTIVE)
                    && lightTimer.seconds() <= 2.0 
                    // && 30 - lightTimer.seconds() >= 4.5 // FIXME: Make this actually representative of rime left
                    && !isPossessingSample(currentSampleDistance)
                )
                && opModeIsActive()
            ) {
                sleep(30); // Waiting whilst freeing CPU for other threads
            }

            // Exiting the retry loop if we don't want to retry
            // if(!retry) {
            //     break retryLoop; 
            // }

            if(isPossessingSample(currentSampleDistance)) {
                break retryLoop;
            }

            // Moving so that we have more chance of getting it.
            hoverSampleAsync();
            setDestinationOffset(retryOffset); // Move from current position forward and a sample up
            lineTo(globalDrive, getCurrentPosition());
            resetDestinationOffset();
        }

        return isPossessingSample(currentSampleDistance);
    }

    /**
     * Performs all necessary actions to deposit the preloaded sample. This 
     * includes moving to the basket, extending the arm, depositing, and a final 
     * retract.
     * 
     * <p> With the exception of an ending arm switch down, this method is 
     * synchronous. 
     */
    private void initialScoreSequence(/*DEV arg*/boolean arg) throws InterruptedException {
        // Getting the arm up and extended (at least partially) first, and the robot in a safe pos
        globalDrive.updatePoseEstimate();
        switchArmAsync(); // Get the arm up
        lift.waitForSwitchStart();

        //#region
        // DEV START: Accumulated pre start stuff 
        final ElapsedTime waitForStartTimer = new ElapsedTime();
        while(!lift.getHasStartedSwitch() && arg && opModeIsActive()) {
            accumulated += "\n\nTimeStamp (sec): " + waitForStartTimer.seconds();
            accumulated += "\nlinearSlideState.name(): " + linearSlideState.name();
            accumulated += "\ngetLinearPivotAvgPos() (DEG): " + AutoArmRunner2.PIVOT_TICKS_PER_DEGREE * getLinearPivotAvgPosition();
            Thread.sleep(200);
        }
        // DEV END
        //#endregion

        queExtendToBucketsAsync(PIVOT_SAFE_FOR_EXTENSION);

        distBack = DIST_BACK;
        distStrafe = DIST_STRAFE;    
        backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0); // don't go too close to the buckets
        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(backAway); // Move back 4 inches to avoid accidental hanging
        moveRobotToNetSafety(isBlue, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-40, 30));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;

        //#region
        // DEV START: During siwtch wait
        accumulated += "\n\n======= Starting switch =======\n\n";
        telemetry.setAutoClear(true);
        telemetry.setMsTransmissionInterval(500);
        final ElapsedTime duringSwitchTimer = new ElapsedTime();
        while(lift.getIsSwitching() && arg && opModeIsActive()) {
            isTelemetrySuppresed = true;
            accumulated += "\n\nTimeStamp (sec): " + duringSwitchTimer.seconds();
            accumulated += "\nlinearSlideState.name(): " + linearSlideState.name();
            accumulated += "\ngetLinearPivotAvgPos() (DEG): " + AutoArmRunner2.PIVOT_TICKS_PER_DEGREE * getLinearPivotAvgPosition();
            
            if(gamepad1.start) {
                telemetry.addLine(accumulated);
                telemetry.addLine();
            }

            if(gamepad1.back) {
                telemetry.addData("linearSlideState.name()", linearSlideState.name());
                telemetry.addData("getLinearPivotAvgPos() (TICKS)", getLinearPivotAvgPosition());
                telemetry.addData("getLinearPivotAvgPos() (DEG)", AutoArmRunner2.PIVOT_TICKS_PER_DEGREE * getLinearPivotAvgPosition());
                // telemetry.addData("", "");
            }

            if(gamepad1.back || gamepad1.start) {
                telemetry.update();
            }
            
            Thread.sleep(200);
        }

        // DEV END
        //#endregion

        // Moving to the basket and scoring when the arm is sufficiently up
        lift.waitForSwitch();

        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(backAway); // Move back 4 inches to avoid accidental hanging
        moveRobotToNetZone(isBlue, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-40, 30));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;

        //#region
        // DEV START: Post switch wait
        accumulated += "\n\n======= Finished switch =======\n\n";
        telemetry.setAutoClear(true);
        telemetry.setMsTransmissionInterval(500);
        while(arg && opModeIsActive()) {
            telemetry.addLine(accumulated);
            telemetry.addLine();
            telemetry.addData("linearSlideState.name()", linearSlideState.name());
            telemetry.addData("getLinearPivotAvgPos() (TICKS)", getLinearPivotAvgPosition());
            telemetry.addData("getLinearPivotAvgPos() (DEG)", AutoArmRunner2.PIVOT_TICKS_PER_DEGREE * getLinearPivotAvgPosition());
            // telemetry.addData("", "");
            telemetry.update();
            Thread.sleep(30);
        }
        // DEV END
        //#endregion

        // Waiting for the arm to be sufficiently extended before depositing
        // Arm is told to lift when teh arm is told to switch up (before the safety move)
        lift.waitForFinish();
        berriddenOfSample(); // Aaaaand deposit, then come back!
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

        distBack = DIST_BACK_LATER - DIST_INCREMENT * (2 - i);
        distStrafe = DIST_STRAFE_LATER;
        backAway = new Pose2d((distBack + distStrafe) / SQRT2, (distStrafe - distBack) / SQRT2, 0); // don't go too close to the buckets
        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(backAway); // Move back to avoid accidental hanging
        moveRobotToNetSafety(isBlue, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-40, 30));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;

        // Moving to the basket and scoring when the arm is sufficiently up
        lift.waitForSwitch();

        MecanumDrive.PARAMS.positionTolerance = NET_ZONE_TOLERANCE;
        setDestinationOffset(backAway); // Move back 4 inches to avoid accidental hanging
        moveRobotToNetZone(isBlue, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-40, 30));
        resetDestinationOffset();
        MecanumDrive.PARAMS.positionTolerance = 1.0;

        // Waiting for the arm to be sufficiently extended before depositing
        // Arm is told to lift when teh arm is told to switch up (before the safety move)
        lift.waitForFinish();
        berriddenOfSample(); // Aaaaand deposit, then come back!
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
        SLIDE_CONSTANTS.intakeEndRetract = 2147000;
        SLIDE_CONSTANTS.depositEndRetract = extendToSampleExtension;

        linearSlideState = LinearSlideStates.INTAKE_FULL;
        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        intakeWheelR.setPower(INTAKE_POWER_HOLD);
        intakeWheelL.setPower(INTAKE_POWER_HOLD);

        // Moving to the buckets, raising the arm, and scoring
        initialScoreSequence(arg);

        // Driving to the spike marks
        boolean isFirstSpikeSample = true;
        grabDepositLoop:
        for(int i = 2; i >= 0 && opModeIsActive(); i--) {
            // Initial positioning data
            final AprilTagDetection spikeMark = getDetection(this.neutralTagId);
            final double extraRotation = i == 0 ? Math.toRadians(90) : 0; // Rotate more cuz' last one's hard to get to. 
            final Pose2d intakeOffset = new Pose2d(-6.25, 0, extraRotation - Math.PI / 2); // Offset from bot center
            final Pose2d grabbingDistance = i == 0 ? new Pose2d(-20, 0, 0) : new Pose2d(-20.0, 0, 0);
            
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
                finalOffset = addPoses(finalOffset, new Pose2d(-2, 5, 0));
                extendToSampleExtension = EXTEND_TO_SAMPLE_LAST_EXTENSION;
            } else {
                finalOffset = addPoses(finalOffset, new Pose2d(1, 1.5, 0));
            }
            
            // Moving to grab the sample
            setDestinationOffset(finalOffset); // Puts the robot into grabbing position
            globalDrive.updatePoseEstimate();
            setRotationType(RotationType.SPLINE);

            // if(isFirstSpikeSample) {
                extendAsync();
            // }

            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            // Grabbing the sample
            resetDestinationOffset();
            grabSampleSync(false, 0); // Grab the pixel. and retract

            if(!isPossessingSample(currentSampleDistance)) {
                continue grabDepositLoop;
            }

            // if(30 - getRuntime() <= 4.0) {
            //     if(30 - getRuntime() >= 1.0) {
            //         goToAscent = true;
            //     }
            //     break grabDepositLoop;
            // }

            retractSync();

            // Waiting for the arm to go up and out of the way
            if(!isPossessingSample(currentSampleDistance)) {
                continue grabDepositLoop;
            }

            scoreSequence(i);
            isFirstSpikeSample = false;
        }

        intakePivot.setPosition(SERVO_VALUES.pivotCarryPos);
        lift.extendSlides(0, 30, -1.0);


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
        
        if(gamepad1.right_trigger > 0.1 && !right_trigger) {grabSampleSync(true, 0); right_trigger = false;} //
        
        if(gamepad1.left_bumper && !left_bumper) {berriddenOfSample(); left_bumper = true;} // Worked, I guess?
        
        if(gamepad1.right_bumper && !right_bumper) {switchArmAsync(); right_bumper = true;} // Worked going into deps; never exited from it UNLESS you depositted        
        
        // if(buttonHandler) {netMoveSync();}
        
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
