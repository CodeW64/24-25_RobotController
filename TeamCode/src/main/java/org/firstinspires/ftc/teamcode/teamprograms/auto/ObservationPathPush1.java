package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config 
@Autonomous(name="Observation Path (Extra Colored Grabber ~ 1 Push)")
public class ObservationPathPush1 extends ObservationPath {
    public static int PUSH_ITER = 1;

    @Override
    protected void main(boolean arg) throws InterruptedException {
        super.main(arg);

        lift.switchArmMode();
        lift.extendSlides(0, RETRACTION_TOLERANCE, RETRACTION_POWER);

        // Parking for 3 points
        resetDestinationOffset();
        lineTo(globalDrive, new Vector2d(-60, -60));
    }
}