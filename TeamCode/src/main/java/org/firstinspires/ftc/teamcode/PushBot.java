////////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;


/**
 * The {@code Pushbot} class allows for all-direction motion and/or rotation.
 * It contains several methods that allow for motion that can easily 
 * be called autonomously and allow for more precise postioning.
 *
 * @version 1.6.0
 * @since {@literal <} Meet 0
 * @see PushbotTeleOp
 */
public class PushBot extends OpMode {
    /** 
     * Radius of the circle that inscribes the robot. Equal to 
     * {@code (feet) (18in / 2 * sqrt(2))}. Change the value if your robot's 
     * size is not 18in. x 18in.
     *  
     * @since {@literal <} Meet 1
     */
    private final double ROBOT_RADIUS = 1.0606601717798213;

    /**
     * Foward speed of the robot when every motor is powered to 0.5. In 
     * feet/second.
     * 
     * <p>Note: This field is only used for {@code moveBlindlyBy} and 
     * {@code moveForwardBlindlyBy}, whose usage is discouraged. If such method
     * is used, then this field should be remeasured and changed to match the 
     * team robot's speed.
     * 
     * <p>Can be measured by holding the A button while running the 
     * {@code PushBotTeleOp} and dividing the distance traveled by the given 
     * time.
     * 
     * @since {@literal <} Meet 0
     */
    private final double HALF_SPEED = 1.7013888888888889;

    /**
     * Rotation speed of the robot when motors are powered to 0.5 or -0.5, 
     * respectively. In radians/second.
     * 
     * @since {@literal <} Meet 0
     */
    private final double ROTATION_SPEED = HALF_SPEED / ROBOT_RADIUS;
    
    private final double TICKS_PER_REVOLUTION = 537.6;
    private final double WHEEL_RADIUS = 0.1574803149606299; // In feet
    private final double DISTANCE_PER_TICK = 2 * Math.PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION; // In feet

    /** 
     * Arbitrary value to make motors more controllable.
     * @since {@literal < } Meet 0
     */
    protected final double MOTOR_TAMING_VALUE = 0.737;
    
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
     * @since {@literal < } Meet 0
     * @see powerDriveMotors()
     */
    protected DcMotor[] driveMotorList = new DcMotor[4];
    
    /** 
     * The internal IMU of the robot. Used to measure velocity (and hence 
     * distance) and also rotation. 
     * 
     * @since {@literal <} State
     */
    protected IMU imu = null;

    /**
     * The last value gotten by {@code getRuntime()}. This should be set at the 
     * end of each call to {@code loop()} if absolute movement is to be used in
     * {@code driveWheels()}.
     * 
     * @since Meet 3
     */
    protected double lastTime = 0;
    protected double totalRotation = 0;

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
     * @version 2.0.2
     * @since {@literal <} Meet 0
     * @see powerDriveMotors()
     */
    protected double[] getDrivePowersFrom(Gamepad gamepad, double speedModifier) {
        // Uses left stick to move (in any direction), and right stick to turn.
        final double x = speedModifier * -gamepad.left_stick_x; // Left/right
        final double y = speedModifier * -gamepad.left_stick_y; // Forward/backward
        final double turnPower = speedModifier * gamepad.right_stick_x;

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
     * Takes a list of powers and applies it to the motors in 
     * {@code motorArray}.
     *
     * @param motorArray DcMotor[] - Motors to be powered (index-to-index with 
     *        {@code powers})
     * @param powers double[] - Powers to be applied to motors (index-to-index with 
     *        {@code motorArray})
     * @version 1.0.1
     * @since {@literal <} Meet 0
     * @see getDrivePowersFrom()
     */
    protected void powerDriveMotors(DcMotor[] motorArray, double[] powers) {
        for(short i = 0; i < motorArray.length; i++) {
            motorArray[i].setPower(powers[i]);
        }
    }
    
    /**
     * Establishes all needed variables and hardware for the methods of 
     * {@code Pushbot}, specifcally the field {@code driveMotorList}.
     * 
     * @version 1.1.0
     * @since {@literal <} Meet 2
     */
    protected void initPushBot() {
        // Setting perferred telemetry settings
         //telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveMotorList[0] = hardwareMap.get(DcMotor.class, "backLeft");
        driveMotorList[1] = hardwareMap.get(DcMotor.class, "frontLeft");
        driveMotorList[2] = hardwareMap.get(DcMotor.class, "backRight");
        driveMotorList[3] = hardwareMap.get(DcMotor.class, "frontRight");
        imu = hardwareMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, 
        // because the axles point in opposite directions.
        driveMotorList[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotorList[1].setDirection(DcMotor.Direction.REVERSE);
        // driveMotorList[2].setDirection(DcMotor.Direction.REVERSE);
        // driveMotorList[3].setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * The default TeleOp {@code loop} operation for child classes. Exists 
     * mainly for better modularization. 
     * 
     * <p>The implemntation can drive the robot using the left stick, or move
     * it absolutely using the dpad.
     * 
     * @version 1.0.0
     * @since {@literal <} Qualifiers
     */
    protected void driveWheels() {
        // Getting the values for absolute direction drive
        final double deltaTime = getRuntime() - lastTime;
        final double up = gamepad1.dpad_up ? -1.0 : 0;
        final double down = gamepad1.dpad_down ? 1.0 : 0;
        final double left = gamepad1.dpad_left ? -1.0 : 0;
        final double right = gamepad1.dpad_right ? 1.0 : 0;
        totalRotation -= 2 * gamepad1.right_stick_x * deltaTime / ROTATION_SPEED;
        
        // Powering the drive motors
        final int fastSpeedFactor = gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0 ? 2 : 1;
        final int slowSpeedFactor = gamepad1.left_bumper || gamepad1.right_bumper ? 2 : 1;
        if(up + down != 0 || left + right != 0) {
        } else {
            // Running the drive motors with relative direction
            final double[] motorPowers = getDrivePowersFrom(
                gamepad1, 
                MOTOR_TAMING_VALUE * fastSpeedFactor / slowSpeedFactor
            );
            powerDriveMotors(driveMotorList, motorPowers);
        }
        
        // Logging important data to telemetry.
        telemetry.setCaptionValueSeparator("");
        telemetry.addData("\n", "=========== Wheel Positions ===========");
        telemetry.setCaptionValueSeparator(": ");
        telemetry.setCaptionValueSeparator(": ");
        for(short i = 0; i < driveMotorList.length; i++) {
            final String[] MOTOR_NAMES = { "Left-Back", "Left-Front", "Right-Back", "Right-Front" };
            telemetry.addData(MOTOR_NAMES[i] + " Position", driveMotorList[i].getCurrentPosition());
        }
        telemetry.addData("Absolute θ", totalRotation);
    }
    
    /**
     * @version 2.0.0
     * @since {@literal <} Meet 0
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