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
            // gamepad2.left_bumper = true;
            sleep(pressDuration);
            // gamepad2.left_bumper = false;
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

    /**
     * Extends the slides to a length of approx 12 inches. 
     * 
     * <p> To use this as a blocking method, call afterwards 
     * lift.waitForFinish(). If that's called, this takes about 2000 ms.
     * 
     * @throws InterruptedException
     */
    private void extendAsync() throws InterruptedException {
        lift.extendSlides(TWELVE_INCHES_EXTENSION, 20, 0.15);
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
        lift.extendSlides(FULLY_RETRACTED, 20, -0.4);
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
        // gamepad2.right_bumper = true;
        gamepad2.right_trigger = 1.0f; // Move to intake position
        sleep(100);
        // gamepad2.right_bumper = false;
        gamepad2.right_trigger = 0;
        // lift.switchToIntakeMode();
    }

    /**
     * Toggles the pivot between intake and deposit mode.
     * 
     * <p> Estimated async completion time: 2000 ms
     * 
     * @throws InterruptedException
     */
    private void toggleArmToDepositAsync() throws InterruptedException {
        gamepad2.left_trigger = 1.0f;
        sleep(100);
        gamepad2.left_trigger = 0;
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
        sleep(100);
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
           (int) AutoArmRunner.SLIDE_CONSTANTS.topBucketHeight, 
           10, 
           0.2
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
        toggleArmToDepositAsync(); // Changing arm position
        sleep(2000);
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
        toggleArmToDepositAsync();
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
        final long SWITCH_TIME = 2500;
        globalDrive.updatePoseEstimate();
        moveRobotToNetZone(isBlue);

        // Putting the preloaded sample into the basket
        switchArmAsync(); // Get the arm up
        sleep(SWITCH_TIME); 
        extendToBucketsAsync(); // Extend there
        lift.waitForFinish();
        netMoveSync(8); // Move forward 8 inches
        berriddenOfSample(); // Aaaaand deposit and return!

        // Driving to the spike marks
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
            setDestinationOffset(finalOffset);
            globalDrive.updatePoseEstimate();
            moveRobotToSpikeMark(spikeMark, i, this.neutralTagId);

            // Grabbing the pixel
            resetDestinationOffset();
            grabSampleSync(); // Grab the pixel. and retract
            retractAsync();
            lift.waitForFinish();
            switchArmAsync(); // Switch the arm up; completed by travel time
            
            // Scoring!
            globalDrive.updatePoseEstimate();
            moveRobotToNetZone(isBlue);
            extendToBucketsAsync();
            lift.waitForFinish();
            netMoveSync(8); // inches 
            berriddenOfSample();
        }

        // Putting the arm into position zero
        linearSlidePivot.setTargetPosition(0);
        linearSlidePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlidePivot.setPower(1.0);

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
            // Thread t = (new Thread() {

            //     @Override
            //     public void run() {
            //         while(true) {
            //             telemetry.addData("Real IMU heading (DEG)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.DEGREES));
            //             telemetry.addData("Real IMU heading (RAD)", globalDrive.bildaDriver.getPosition().getHeading(AngleUnit.RADIANS));
            //             telemetry.addLine();
            //             telemetry.addData("Real IMU heading Vel (DEG)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.DEGREES));
            //             telemetry.addData("Real IMU heading Vel (RAD)", globalDrive.bildaDriver.getVelocity().getHeading(AngleUnit.RADIANS));
            //             telemetry.update();
            //         }
            //     }
            // });
            // t.setDaemon(true);
            // t.start();
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
