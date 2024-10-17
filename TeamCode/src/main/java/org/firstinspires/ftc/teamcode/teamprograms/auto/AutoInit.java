package org.firstinspires.ftc.teamcode.teamprograms.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;


/**
 * The {@code Pushbot} class allows for all-direction motion and/or rotation.
 * It contains several methods that allow for motion that can easily 
 * be called autonomously and allow for more precise postioning.
 */
public class AutoInit extends OpMode {
    /** 
     * Arbitrary value to make motors more controllable.
     */
    // protected final double MOTOR_TAMING_VALUE = 0.737;
    
    /**
     * A list of all the motors. Rarely ever used outside of an argument in 
     * {@code powerDriveMotors()} 
     * 
     * <p>The index order is as follows:
     * 
     * <p>{@code 0} - {@code left_drive_backward}
     * 
     * <p>{@code 1} - {@code left_drive_forward}
     * 
     * <p>{@code 2} - {@code right_drive_backward}
     * 
     * <p>{@code 3} - {@code right_drive_forward}
     * 
     * @see powerDriveMotors()
     */
    protected DcMotorEx[] driveMotorList = new DcMotorEx[4];
    
    /** 
     * The internal IMU of the robot. Used to measure velocity (and hence 
     * distance) and also rotation. 
     * 
     */
    protected IMU imu = null;

    /**
     * Gets the power for each wheel's motor from the gamepad object. 
     * 
     * <h2>Controls</h2>
     * 
     * <p>left stick - Makes the robot move in some direction relative to its
     * rotation
     * 
     * <p>right stick - Rotates the robot
     * 
     * @param gamepad Gamepad - Used to derive respective powers and direction.
     * @param speedModifier double - A multiplier to both driving and turning speed.
     * @return double[] - Powers for each motor to achieve the action 
     *         described by the controller.
     * @see powerDriveMotors()
     */
    protected double[] getDrivePowersFrom(Gamepad gamepad, double speedModifier) {
        // Uses left stick to move (in any direction), and right stick to turn.
        final double x = speedModifier * -gamepad1.left_stick_x; // Left/right
        final double y = speedModifier * -gamepad1.left_stick_y; // Forward/backward
        final double turnPower = speedModifier * gamepad1.right_stick_x;

        // POWER_FACTOR == sqrt(0.5). sqrt(0.5) comes from trig and is used to make it move in any direction. 
        // Both plus and minus exist because of how mechanum wheels rotate to move sideways
        final double POWER_FACTOR = 0.7071067811865476; 
        final double drivePowerPlus  = POWER_FACTOR * (y + x);
        final double drivePowerMinus = POWER_FACTOR * (y - x);
      
        double[] powers = new double[4];
        powers[0] = Range.clip(drivePowerPlus  + turnPower, -1.0, 1.0);
        powers[1] = Range.clip(drivePowerMinus + turnPower, -1.0, 1.0);
        powers[2] = Range.clip(drivePowerMinus - turnPower, -1.0, 1.0);
        powers[3] = Range.clip(drivePowerPlus  - turnPower, -1.0, 1.0);
        return powers;
    }

    /**
     * Gets the power for each wheel's motor from the given directions. 
     * 
     * <h2>Controls</h2>
     * 
     * <p>left stick - Makes the robot move in some direction relative to its
     * rotation
     * 
     * <p>right stick - Rotates the robot
     * 
     * @param vx double - Side-to-side movement, in interval [-1, 1], to move. 
     *           Negative is left, positive is right.
     * @param vy double - Forward-back movement, in interval [-1, 1], to move. 
     *           Negative is backwards, positive is forwards.
     * @param turn double - How much to turn, in interval [-1, 1]. Negative is 
     *             CCW; postive is CW.
     * @param speedModifier double - A multiplier to both driving and turning speed.
     * @return double[] - Powers for each motor to achieve the action 
     *         described by the controller.
     * @see powerDriveMotors()
     */
    protected double[] getDrivePowersFrom(double vx, double vy, double turn, double speedModifier) {
        // Uses left stick to move (in any direction), and right stick to turn.
        final double x = speedModifier * -gamepad1.left_stick_x; // Left/right
        final double y = speedModifier * -gamepad1.left_stick_y; // Forward/backward
        final double turnPower = speedModifier * gamepad1.right_stick_x;

        // POWER_FACTOR == sqrt(0.5). sqrt(0.5) comes from trig and is used to make it move in any direction. 
        // Both plus and minus exist because of how mechanum wheels rotate to move sideways
        final double POWER_FACTOR = 0.7071067811865476; 
        final double drivePowerPlus  = POWER_FACTOR * (y + x);
        final double drivePowerMinus = POWER_FACTOR * (y - x);
      
        double[] powers = new double[4];
        powers[0] = Range.clip(drivePowerPlus  + turnPower, -1.0, 1.0);
        powers[1] = Range.clip(drivePowerMinus + turnPower, -1.0, 1.0);
        powers[2] = Range.clip(drivePowerMinus - turnPower, -1.0, 1.0);
        powers[3] = Range.clip(drivePowerPlus  - turnPower, -1.0, 1.0);
        return powers;
    }
    
    /**
     * Pushes the robot along using the specified vector endicitive of a 
     * gamepad's {@code left_stick_x} and {@code left_stick_y} properties. 
     * 
     * <p>Note: distances are only approximate to real life.
     * 
     * <p>Note: distances are only approximate to real life. It depends on the 
     * accuracy of the {@code WHEEL_DIAMETER} and {@code TICKS_PER_REVOLUTION} 
     * variables and any slipping of the wheels. 
     *
     * @param vx double - The distance left (+) or right (-) in inches
     * @param vy double - The distance forward (+) or backward (-) in inches
     */
    protected void moveBy(double vx, double vy) {
        // TODO: Fix the moveBy method so that it uses odometry
        // // Sleeping to give time for resetting. Try removing it to see why.
        // try {
        //     Thread.sleep(500);
        // } catch(InterruptedException err) {
        //     return;
        // }
        
        // // Finding the powers appropriate for the specified movement
        // final Gamepad fakeGamepad = new Gamepad();
        // final double distance = Math.hypot(vx, vy);
        // fakeGamepad.left_stick_x = (float) (vx / distance); // Dividing distance normalizes the vector
        // fakeGamepad.left_stick_y = (float) (vy / distance); // Dividing distance normalizes the vector
        // double[] powers = getDrivePowersFrom(fakeGamepad, 1);

        // // Setting targets
        // for(short i = 0; i < powers.length; i++) {
        //     final DcMotorEx motor = driveMotorList[i];
        //     final double power = powers[i];
        //     motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //     motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //     motor.setTargetPosition((int) (power * distance / DISTANCE_PER_TICK));
        //     motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //     /* 
        //      * Because RUN_TO_POSITION always goes DIRECTLY to the target when the 
        //      * sign is positive (and this being the disired operation), we always 
        //      * make sure it is positive.
        //      * 
        //      * The value is halved to reduce any problems of high speeds. 
        //      */
        //     powers[i] = 0.25 * Math.abs(power);
        //     telemetry.addData("power" + i, power);
        // }
        // telemetry.update();

        // // Driving the motors. 
        // for(DcMotorEx motor : driveMotorList) {
        //     motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // }
        // powerDriveMotors(powers);
        // while(
        //     driveMotorList[0].isBusy() 
        //     && driveMotorList[1].isBusy() 
        //     && driveMotorList[2].isBusy() 
        //     && driveMotorList[3].isBusy()
        // ) {
        //     // Just waiting...
        // }

        // // Braking the motors
        // for(DcMotorEx motor : driveMotorList) {
        //     motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // }
        // powerDriveMotors(new double[]{0, 0, 0, 0});
    }
    
    // TODO: Fix the moveForward method so that it uses odometry
    /**
     * Pushes the robot along {@code distance} inches forward. Using a negative 
     * number moves the robot backwards.
     * 
     * <p>Note: This method requires that all wheels' motors possess encoders.
     * 
     * <p>Note: distances are only approximate to real life. It depends on the 
     * accuracy of the {@code WHEEL_DIAMETER} and {@code TICKS_PER_REVOLUTION} 
     * variables and the relaibility of the wheels to travel a distance close to
     * {@code distance}. 
     *
     * @param distance double - The distance forward to move in inches.
     * @see moveBy()
     */
    protected void moveForwardBy(double distance) {
        moveBy(0, distance); 
    }
    
    /**
     * Rotates the robot by the specified amount. Units are in radians 
     * counterclockwise. 
     * 
     * @param theta double - Angle measure to rotate counterclockwise (+) or 
     *        clockwise (-) in radians
     */
    protected void rotateBy(double theta) {
        // TODO: Change rotate by to use odometry
        // // Sleeping to give time for resetting. Try removing it to see why.
        // try {
        //     Thread.sleep(500);
        // } catch(InterruptedException err) {
        //     return;
        // }
        
        // final double startRotation = imu.getRobotYawPitchRoll().getYaw(AngleUnit.DEGREES);
    }
    
    /**
     * Takes a list of powers and applies it to the motors in 
     * {@code driveMotorList}.
     * 
     * @param powers double[] - Powers to be applied to motors (index-to-index with 
     *        {@code motorArray})
     * @see getDrivePowersFrom()
     */
    protected void powerDriveMotors(double[] powers) {
        for(short i = 0; i < driveMotorList.length; i++) {
            driveMotorList[i].setPower(powers[i]);
        }
    }

    /**
     * Takes powers and applies it to the motors in {@code driveMotorList}.
     * 
     * <p> TODO: Put parameter descriptions into this method's JavaDoc
     * @see getDrivePowersFrom()
     */
    protected void powerDriveMotors(double leftBack, double leftForw, double rightBack, double rightForw) {
        driveMotorList[0].setPower(leftBack);
        driveMotorList[1].setPower(leftForw);
        driveMotorList[2].setPower(rightBack);
        driveMotorList[3].setPower(rightForw);
    }
    
    /**
     * Establishes all needed variables and hardware for the methods of 
     * {@code Pushbot}, specifcally the field {@code driveMotorList}.
     * 
     */
    protected void initPushBot() {
        // Setting perferred telemetry settings
         //telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveMotorList[0] = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveMotorList[1] = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveMotorList[2] = hardwareMap.get(DcMotorEx.class, "backRight");
        driveMotorList[3] = hardwareMap.get(DcMotorEx.class, "frontRight");
        imu = hardwareMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, 
        // because the axles point in opposite directions.
        driveMotorList[0].setDirection(DcMotorEx.Direction.REVERSE);
        driveMotorList[1].setDirection(DcMotorEx.Direction.REVERSE);
        // driveMotorList[2].setDirection(DcMotorEx.Direction.REVERSE);
        // driveMotorList[3].setDirection(DcMotorEx.Direction.REVERSE);
    }

    /**
     * The default TeleOp {@code loop} operation for child classes. Exists 
     * mainly for better modularization. 
     * 
     * <p>The implemntation can drive the robot using the left stick, or move
     * it absolutely using the dpad.
     * 
     */
    protected void driveWheels() {
        // Getting the values for absolute direction drive
        final double up = gamepad1.dpad_up ? -1.0 : 0;
        final double down = gamepad1.dpad_down ? 1.0 : 0;
        final double left = gamepad1.dpad_left ? -1.0 : 0;
        final double right = gamepad1.dpad_right ? 1.0 : 0;
        
        // Powering the drive motors
        final int fastSpeedFactor = gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0 ? 2 : 1;
        final int slowSpeedFactor = gamepad1.left_bumper || gamepad1.right_bumper ? 2 : 1;
        // Running the drive motors with relative direction
        final double[] motorPowers = getDrivePowersFrom(
            gamepad1, 
            fastSpeedFactor / slowSpeedFactor
        );
        powerDriveMotors(motorPowers);
        
        // Logging important data to telemetry.
        telemetry.setCaptionValueSeparator("");
        telemetry.addData("\n", "=========== Wheel Positions ===========");
        telemetry.setCaptionValueSeparator(": ");
        telemetry.setCaptionValueSeparator(": ");
        for(short i = 0; i < driveMotorList.length; i++) {
            final String[] MOTOR_NAMES = { "Left-Back", "Left-Front", "Right-Back", "Right-Front" };
            telemetry.addData(MOTOR_NAMES[i] + " Position", driveMotorList[i].getCurrentPosition());
        }
    }
    
    /**
     */
    @Override
    public void init() {
        initPushBot();
    }

    @Override
    public void loop() {
        // Meant to be overridden.
        // 🎶🎷🐛
    }
}