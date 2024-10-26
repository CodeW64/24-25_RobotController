package org.firstinspires.ftc.teamcode.gobilda;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.messages.GoBildaFullInputsMessage;
import org.firstinspires.ftc.teamcode.messages.RRGoBildaInputsMessage;


@Config
public final class GoBildaFullLocalizer implements Localizer {
    public static class Params {

        // 30 = 2004.5
        public double parYTicks = 2004.5; // y position of the parallel encoder (in tick units)

        // 30 = 130.1
        public double perpXTicks = 130.1; // x position of the perpendicular encoder (in tick units)

    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;

    public final GoBildaPinpointDriver bildaDriver; // this replaces the REV internal IMU
    //    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private Twist2dDual<Time> lastTwist;

    private final double inPerTick;

    private boolean initialized;

    public GoBildaFullLocalizer(HardwareMap hardwareMap, GoBildaPinpointDriver bildaDriver, double inPerTick) {

        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backRight")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRight")));

        this.bildaDriver = bildaDriver;

        bildaDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        bildaDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        bildaDriver.resetPosAndIMU(); // should let robot sit still for 0.25s

        this.inPerTick = inPerTick;

        FlightRecorder.write("RR_GOBILDA_PARAMS", PARAMS);

    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

//        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
//        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);


//        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));


        bildaDriver.update();
//        Pose2D bildaPos = bildaDriver.getPosition();
//        Pose2D bildaVel = bildaDriver.getVelocity();
        int bildaXPos = bildaDriver.getEncoderX();
        int bildaYPos = bildaDriver.getEncoderY();
        double bildaHeading = bildaDriver.getHeading();
        // TODO: x and y return mm/s velocity; make it raw value
        double bildaXVel = bildaDriver.getVelX();
        double bildaYVel = bildaDriver.getVelY();
        double bildaHeadingVel = bildaDriver.getHeadingVelocity();

        FlightRecorder.write("GOBILDA_FULL_INPUTS", new GoBildaFullInputsMessage(bildaXPos,
                bildaYPos, bildaHeading, bildaXVel, bildaYVel, bildaHeadingVel));


//        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        double currentHeadingAngle = bildaHeading;
        currentHeadingAngle = Math.floor(currentHeadingAngle * 100) / 100;
        Rotation2d heading = Rotation2d.exp(currentHeadingAngle);


        double headingVel = (float) bildaHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            lastTwist = new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }




        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        if (Double.isNaN(headingDelta) || Double.isNaN(headingVel)) {
            return lastTwist;
        }


        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;
        lastTwist = twist;

        return twist;
    }
}