package org.firstinspires.ftc.teamcode.teamprograms;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamprograms.DistanceGetter;

public class AscentStabilizer {
    /*DistanceUnit*/ public DistanceUnit dUnit;
    /*DistanceSensor*/ public DistanceGetter sensor;

    // Mathematical constants 
    // NOTE: These are not hard and fast constants and may be changed outside of the class!
    //       Please refer to the instance reference to see the constants and for any changes.
    public double h = 35.5;  // How far the second rung is off the ground.
    public double x = 14.25; // How far the pivot's from the rungs
    public double r = 1.25;  // Radius of the hook.

    // Other mathematical variables
    public boolean firstUpdate = true;
    public double lastY = 0;
    public double lastT = 0;
    public double currentT = this.lastT;
    public double currentY = this.lastY;

    public AscentStabilizer(DistanceGetter sensor, double startTimeStamp) {
        this.dUnit = DistanceUnit.INCH;
        this.sensor = sensor;
        this.update(startTimeStamp); // Sets the lastT/Y and currentT/Y fields
    }

    public AscentStabilizer(DistanceGetter sensor) {
        this.dUnit = DistanceUnit.INCH;
        this.sensor = sensor;
        this.update(0); // Sets the lastT/Y and currentT/Y fields
    }

    /**
     * Returns how far the pivot is off the ground. To get an accurate 
     * reading, the distance sensor must be accurate to range, and the 
     * height sensor offset describes how far the sensor is above the pivot
     * (in this stabilizer's distance units). 
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return How far the pivot is off the ground, in this stabilizer's 
     *     distance units.
     */
    public double y(double t) {
        if(t == currentT) {
            // Same input, same output
            return currentY;
        }

        // New input, new ouput
        return sensor.getDistance(dUnit);
    }

    /**
     * Returns the vertical distance between the pivot and the second rung. 
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return Difference between the second rung and pivot, in the object's 
     *     distance units.
     */
    public double deltaY(double t) {
        return h - y(t);
    }

    /**
     * Returns the (discrete) derivative of the y coordinates. In other 
     * words, the slope of the the last two Ys is found over the delta time.
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return Difference between the last y and the current y, divided by 
     *     difference in time between the measurements. 
     */
    public double yPrime(double t) {
        if(y(t) - lastY == 0) {
            return 0; // Used when the denominator is 0
        }

        return (y(t) - lastY) / (t - lastT);
    }

    /**
     * Computes the angle at which the hook will be hooked onto the second 
     * rung.
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return Desired angle of the pivot, in radians
     */
    public double theta(double t) {
        final double u = deltaY(t);
        final double squaresSum = u * u + x * x;
        return Math.asin((-r * x + Math.abs(u) * Math.sqrt(squaresSum - r * r)) / squaresSum);
    }

    /**
     * Computes the length at which the arm will hook onto the second rung. 
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return Desired length of the pivot, in the object's distance units
     */
    public double l(double t) {
        final double theta = theta(t);
        return x * Math.cos(theta) + deltaY(t) * Math.sin(theta);
    }

    /**
     * Computes the first derivative of the theta. This is based on the 
     * current desired theta, current height, and the height vel, not the 
     * last theta.
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return First derivative of theta position at the given timestamp, in 
     *     the object's distance units per the object's time units.
     */
    public double thetaPrime(double t) {
        final double squaresSum = deltaY(t) * deltaY(t) + x * x;
        return (
            -(1 / Math.cos(theta(t)))
            * yPrime(t)
            / (squaresSum * squaresSum)
            * (
                Math.abs(deltaY(t)) / deltaY(t)
                * (
                    Math.pow(x, 4)
                    + x * x * (deltaY(t) * deltaY(t) - r * r)
                    + r * r * deltaY(t) * deltaY(t)
                )
                / Math.sqrt(squaresSum - r * r)
                + 2 * deltaY(t) * r * x
            )
        );
    }

    /**
     * Computes the first derivative of the length. This is based on the 
     * current desired theta, the theta vel, current height, and the height 
     * vel, not the last length.
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     * @return First derivative of length position at the given timestamp, 
     *     in the object's distance units per the object's time units.
     */
    public double lPrime(double t) {
        final double theta = theta(t);
        final double sine = Math.sin(theta);
        return (
            thetaPrime(t) 
            * (deltaY(t) * Math.cos(theta) - x * sine) 
            - yPrime(t) * sine
        );
    }

    /**
     * Updates the lastT,  lastY, and pitch fields of the object. This must
     * be called everytim before other methods of the object to get accurate 
     * values. The pitch argument is used to adjust the height, which would 
     * be altered if the robot was tilting.
     * 
     * @param t Timestamp the function was called at. In the time units 
     *     given in the constructor.
     */
    public void update(double t) {
        // Setting the last properties for derivatives
        if(firstUpdate) {
            // Apply no average so that placeholder values aren't factored in
            lastY = currentY = y(t);
            lastT = currentT = t;
            firstUpdate = false;
        }

        lastY = currentY;
        lastT = currentT;

        // Setting the current properties
        currentY = y(t);
        currentT = t;
    }
}
