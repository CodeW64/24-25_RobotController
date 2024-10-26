package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class GoBildaFullInputsMessage {
    public long timestamp;
    public int xPos;
    public int yPos;
    public double xVel;
    public double yVel;
    public double yaw;
    public double zRotationRate;

    public GoBildaFullInputsMessage(int bildaXPos, int bildaYPos, double bildaHeading,
                                    double bildaXVel, double bildaYVel, double bildaHeadingVel) {
        this.timestamp = System.nanoTime();

        this.xPos = bildaXPos;
        this.yPos = bildaYPos;
        this.xVel = bildaXVel;
        this.yVel = bildaYVel;
        this.yaw = bildaHeading;
        this.zRotationRate = bildaHeadingVel;

    }
}
