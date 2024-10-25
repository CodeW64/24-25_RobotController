package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class RRGoBildaInputsMessage {
    public long timestamp;
    public PositionVelocityPair par;
    public PositionVelocityPair perp;
    public double yaw;
    public double zRotationRate;

    public RRGoBildaInputsMessage(PositionVelocityPair par, PositionVelocityPair perp, double headingAngle, double headingVel) {
        this.timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
        {
            this.yaw = headingAngle;
        }
        {
            this.zRotationRate = headingVel;
        }
    }
}
