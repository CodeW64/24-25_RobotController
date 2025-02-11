package org.firstinspires.ftc.teamcode.teamprograms.temp;

import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamprograms.auto.AutoInit;
import org.firstinspires.ftc.teamcode.teamprograms.ButtonPressHandler;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

@TeleOp(group="AAA")
@Config
public class Robot2TestSampleSensor extends LinearOpMode {

    ColorRangeSensor sampleSensor;

    public static class SensorVariables {
        public double blueMaxHue = 250;
        public double blueMinHue = 180;
        public double maxColorDistCm = 6;
        public double minSaturation = 0.3;
        public double redMaxHue = 80;
        public double redMinHue = 0;
        public double neutralMaxHue = 120;
        public double neutralMinHue = 80;
    }

    public static SensorVariables SENSOR_VARIABLES = new SensorVariables();

    public static enum Alliance { 
        NEUTRAL, 
        BLUE, 
        RED, 
        UNKNOWN 
    }

    public static int TRANSMISSION_MS_INTERVAL = 33;
    public static int SLEEP_TIME = 33;
    public static double GAIN = 200.0;
    public static boolean USE_NORM = false;
    public static boolean USE_BITMAP = false;

    public void runOpMode() {
        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sampleSensor");

        telemetry.setAutoClear(true);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        boolean isPaused = false;
        boolean isLedOn = true;
        String previousLog = "";
        ButtonPressHandler aButton = null;
        ButtonPressHandler bButton = null;

        try {
            aButton = new ButtonPressHandler(gamepad1, "a");
            bButton = new ButtonPressHandler(gamepad1, "b");
        } catch(NoSuchFieldException err) {
            // Nothing to do; the null-ness of the handler is reported in the main loop.
        }

        opModeLoop:
        while(opModeIsActive()) {
            // Controlling the telemetry from user input and dashboard changes
            try {
                if(aButton == null) {
                    telemetry.addLine("============= NOTICE =============");
                    telemetry.addLine("The <A> ButtonPressHandler is null");
                    telemetry.addLine("==================================\n");
                } else if (aButton.isPressed()) {
                    isPaused = !isPaused;
                }

                if(bButton == null) {
                    telemetry.addLine("============= NOTICE =============");
                    telemetry.addLine("The <A> ButtonPressHandler is null");
                    telemetry.addLine("==================================\n");
                } else if(bButton.isPressed()) {
                    isLedOn = !isLedOn;
                    sampleSensor.enableLed(isLedOn);
                }
            } catch(IllegalAccessException err) {
                telemetry.addData("!!!ButtonHandler Exception", err);
            }

            // Skipping this loop if the opmode is paused
            if(isPaused) {
                try {
                    Thread.sleep(SLEEP_TIME);
                } catch(InterruptedException err) {
                    // Nothing important to do; this is normal and expected
                }
                continue opModeLoop;
            }

            telemetry.setMsTransmissionInterval(TRANSMISSION_MS_INTERVAL);

            // Getting the data
            final double dist = sampleSensor.getDistance(DistanceUnit.CM);
            int argb = 0;
            int red = sampleSensor.red();
            int green = sampleSensor.green();
            int blue = sampleSensor.blue();

            sampleSensor.setGain((float) GAIN);
            if(USE_NORM) {
                red   = (int) (255 * sampleSensor.getNormalizedColors().red);
                green = (int) (255 * sampleSensor.getNormalizedColors().green);
                blue  = (int) (255 * sampleSensor.getNormalizedColors().blue);
            }

            argb = sampleSensor.argb();
            if(USE_BITMAP) {
                red   = (argb >> 16) & 0xff;
                green = (argb >>  8) & 0xff;
                blue  = (argb      ) & 0xff;
            }

            final double hue = JavaUtil.rgbToHue(red, green, blue);
            final double saturation = JavaUtil.rgbToSaturation(red, green, blue);
            final double value = JavaUtil.rgbToValue(red, green, blue);
            
            final Alliance alliance = determineAlliance(hue, saturation, dist);

            // Formatting the data
            final String format = 
                "<p>Distance (cm): %.2f</p>" + 
                "<br/>" + 

                "<h2>RGB Color</h2>" + 
                "<p style=\"font-family:monospace;\">Red: %d</p>" +
                "<p style=\"font-family:monospace;\">Green: %d</p>" +
                "<p style=\"font-family:monospace;\">Blue: %d</p>" +
                "<p style=\"font-familt:monospace;\">ARGB: #%08x</p>" +
                "<br/>" +

                "<h2>Normalized Color</h2>" + 
                "<p style=\"font-family:monospace;\">Red: %.3f</p>" +
                "<p style=\"font-family:monospace;\">Green: %.3f</p>" +
                "<p style=\"font-family:monospace;\">Blue: %.3f</p>" +
                "<br/>" +

                // "\n <h2>Swatch</h2>" + 
                // "\n <p style=\"width:40px;height:40px;background-color:#%02X%02X%02X\"></p>" +
                // "\n <br/>" +

                "<h2>HSV Color</h2>" +
                "<p style=\"font-family:monospace;\">Hue: %.1f</p>" + 
                "<p style=\"font-family:monospace;\">Saturation: %.2f</p>" + 
                "<p style=\"font-family:monospace;\">Value: %.2f</p>" +
                "<br/>" + 

                "<h2>Alliance</h2>" +
                "<p style=\"font-family:monospace;\">Alliance: %s</p>" + 
                "<br/>" + 

                "<h2>Misc</h2>" +
                "<p>Is Paused: %b</p>" +
                "<p>Is LED On: %b</p>" 
                ;

            previousLog = String.format(
                format, 

                dist,

                // RGB
                red, 
                green, 
                blue, 
                argb,
                
                // SWATCH
                sampleSensor.getNormalizedColors().red, 
                sampleSensor.getNormalizedColors().green, 
                sampleSensor.getNormalizedColors().blue,

                // HSV
                hue,
                saturation,
                value,

                // ALLIANCE
                alliance.name(),

                // MISC
                isPaused,
                isLedOn
            );

            // Logging the data
            telemetry.addLine(previousLog);
            telemetry.update();
        }
    }

    protected Alliance determineAlliance(double hue, double sat, double distCm) {
        // If the color cannot be safely determined
        if(sat < SENSOR_VARIABLES.minSaturation || distCm >= SENSOR_VARIABLES.maxColorDistCm) {
            return Alliance.UNKNOWN;
        } 

        final boolean isYellow = SENSOR_VARIABLES.neutralMinHue <= hue && hue <= SENSOR_VARIABLES.neutralMaxHue;
        final boolean isRed    = SENSOR_VARIABLES.redMinHue     <= hue && hue <= SENSOR_VARIABLES.redMaxHue;
        final boolean isBlue   = SENSOR_VARIABLES.blueMinHue    <= hue && hue <= SENSOR_VARIABLES.blueMaxHue;

        // if(this.isRomania) {
        //     this.isRomania = false;
        //     this.isChad = true;
        // }

        // Check for nonsense and possibly return neutral
        if(isYellow) {
            if(isRed || isBlue) {
                // Two or more colors were matched; the color is nonsense
                return Alliance.UNKNOWN;
            }

            return Alliance.NEUTRAL;
        }
        
        // Check for nonsense and possibly return red
        if(isRed) {
            if(isBlue || isYellow) {
                // Two or more colors were matched; the color is nonsense
                return Alliance.UNKNOWN;
            }

            return Alliance.RED;
        }

        // Check for nonsense and possibly return blue
        if(isBlue) {
            if(isRed || isYellow) {
                // Two or more colors were matched; the color is nonsense
                return Alliance.UNKNOWN;
            }

            return Alliance.BLUE;
        }

        // The color was unrecognized; the color is unknown
        return Alliance.UNKNOWN;
    }
}