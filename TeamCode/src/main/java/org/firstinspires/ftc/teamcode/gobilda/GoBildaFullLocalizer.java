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

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.messages.GoBildaFullInputsMessage;

/*
    IMPORTANT: LOCALIZER NOT FUNCTIONAL
    localizer fully abandons roadrunner's way of grabbing pose
    in favor of the goBILDA pinpoint odometry computer
 */

@Config
@Deprecated
public final class GoBildaFullLocalizer implements Localizer {
    public static class Params {

        // 30 = 2004.5
        public double parYTicks = 2004.5; // y position of the parallel encoder (in tick units)

        // 30 = 130.1
        public double perpXTicks = 130.1; // x position of the perpendicular encoder (in tick units)

    }

    public static Params PARAMS = new Params();


    public final GoBildaPinpointDriver bildaDriver; // this replaces the REV internal IMU and encoders

    public boolean hasReturnedNaN = false;
    private final float SWINGARM_RESOLUTION = 13.26291192f;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private Twist2dDual<Time> lastTwist;

    private final double inPerTick;

    private boolean initialized;

    public GoBildaFullLocalizer(HardwareMap hardwareMap, GoBildaPinpointDriver bildaDriver, double inPerTick) {

        this.bildaDriver = bildaDriver;

        this.bildaDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); // so velocity outputs are raw
        // VELOCITY OUTPUTS IN THE 100s

        this.bildaDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.bildaDriver.resetPosAndIMU(); // should let robot sit still for 0.25s

        this.inPerTick = inPerTick;

        FlightRecorder.write("RR_GOBILDA_PARAMS", PARAMS);

    }

    public Twist2dDual<Time> update() {

        // get pinpoint driver pose/velocity data
        bildaDriver.update();
        int bildaParXPos = bildaDriver.getEncoderX();
        int bildaPerpYPos = bildaDriver.getEncoderY();
        double bildaHeading = bildaDriver.getHeading();
        double bildaParXVel = (int) bildaDriver.getVelX();
        double bildaPerpYVel = (int) bildaDriver.getVelY();
        double bildaHeadingVel = bildaDriver.getHeadingVelocity();

        FlightRecorder.write("GOBILDA_FULL_INPUTS", new GoBildaFullInputsMessage(bildaParXPos,
                bildaPerpYPos, bildaHeading, bildaParXVel, bildaPerpYVel, bildaHeadingVel));



        double currentHeadingAngle = bildaHeading;
        currentHeadingAngle = Math.floor(currentHeadingAngle * 100) / 100;
        Rotation2d heading = Rotation2d.exp(currentHeadingAngle);


        double headingVel = (float) bildaHeadingVel;

        if (!initialized) {
            initialized = true;


            lastParPos = bildaParXPos;
            lastPerpPos = bildaPerpYPos;
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




        int parPosDelta = bildaParXPos - lastParPos;
        int perpPosDelta = bildaPerpYPos - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        if (Double.isNaN(headingDelta) || Double.isNaN(headingVel)) {
            hasReturnedNaN = true;
            return lastTwist;
        }


        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                bildaParXVel - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                bildaPerpYVel - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = bildaParXPos;
        lastPerpPos = bildaPerpYPos;
        lastHeading = heading;
        lastTwist = twist;

        return twist;
    }
}