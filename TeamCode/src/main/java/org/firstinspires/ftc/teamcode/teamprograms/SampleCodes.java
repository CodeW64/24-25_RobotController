package org.firstinspires.ftc.teamcode.teamprograms;

public class SampleCodes {
    double red;
    double green;
    double blue;

    public SampleCodes(double red, double green, double blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }


    public double[] getColorCodes() {
        return new double[] {red, green, blue};
    }
}
