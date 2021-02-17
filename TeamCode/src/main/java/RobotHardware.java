/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class RobotHardware
{
    public DcMotor  leftFrontDrive  = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  leftRearDrive  = null;
    public DcMotor  rightRearDrive = null;

    public DcMotor  wobbleMotor = null;

    public DcMotor leftLauncherMotor = null;
    public DcMotor rightLauncherMotor = null;

    public DcMotor scoopMotor = null;

    public Servo wobbleServo = null;
    public Servo wobbleServoTwoProng = null;

    public Servo launchServo = null;

    public Servo leftGrabberServo = null;
    public Servo rightGrabberServo = null;

    public Servo liftServo = null;

    BNO055IMU imu;

    public Rev2mDistanceSensor rearDistanceSensor;
    public Rev2mDistanceSensor rearDistanceSensorTwo;
    public Rev2mDistanceSensor frontDistanceSensor;
    public Rev2mDistanceSensor frontDistanceSensorTwo;

    HardwareMap hwMap           =  null;

    //Hardware constants

    public static final double TELEOPDEADZONE = 0.05;
    public static final double NOTTURBOFACTOR = 0.5;
    public static final int CLICKS_PER_INCH = 45;
    public static final int STRAFE_CLICKS_PER_INCH = 48;

    public final double HIGH_TURN_POWER = 0.3;
    public final double MEDIUM_TURN_POWER = 0.12;
    public final double LOW_TURN_POWER = 0.1;

    public final double WOBBLE_SERVO_OPEN = 0.78;
    public final double WOBBLE_SERVO_CLOSED = 1.00;

    public final double WOBBLE_SERVO_TWO_PRONG_OPEN = 0.22;
    public final double WOBBLE_SERVO_TWO_PRONG_CLOSED = 0;

    public final int SCOOPING_POSITION = 0;
    public final int RING_SERVO_BARELY_LIFTED = 150;
    public final int DROPPING_POSITION = 1825;
    public final int HEX_CLICKS_PER_REVOLUTION = 288;
    public final int DRIVING_POSITION = 1575;

    public final long GRAB_TIMER = 400;
    public final long RELEASE_TIMER = 100;

    public final double LEFT_GRABBER_CLOSED = 0.23;
    public final double RIGHT_GRABBER_CLOSED = 0.69;
    public final double LEFT_GRABBER_OPEN = 0.6;
    public final double RIGHT_GRABBER_OPEN = 0.3;

    public final double LAUNCHER_FIRE_POSITION = 0;
    public final double LAUNCHER_RESET_POSITION = 1;
    public final double LAUNCHER_HIGH_ANGLE = 0.761;
    public final double LAUNCHER_POWER_ANGLE = 0.73;

    public final double LAUNCHER_LOAD_ANGLE = 0.59;

    public final int WOBBLE_ARM_DOWN = 1200;
    public final int WOBBLE_ARM_UP = 600;
    double voltage;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AV5lMdL/////AAABmYl8p4yeaEBpg80chUBr03OEyO2uvBZSdFt80EPGeZY6RLNOxd+um0wYnUnvUtSKSRGDHPxjUybk/" +
                    "5S3xKgJn0vYHVl5OhDJeo75MxhpDax25TizaBl/eJ19okrNQV2D8DYyURQktmaETVqx5X2GL4SNmkovGNQV6O" +
                    "NlcHWfyuyyhP/+eQx3JgLqPKSD9lWkLEf9Wc3D1k2N9o1EfzpvOVv+jazDnUjGOVy+wrATGq5H8Tk2VVNryzl" +
                    "ZZ4qoD5JOzyAvDKrOmH/3V/qk77SdOONeVbokFFc8ErD4S+cf/EkqjBxZTwzjyT0P/0BSqwWjz7716YeEJ0C9" +
                    "o7dkNNbJ/AkGDf51lt1VFzSmPtldip8h";
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public double shooterWheelSpeed = 0.97;

    public RobotHardware(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        wobbleServoTwoProng = hwMap.get(Servo.class,"wobbleServoTwoProng");
        wobbleMotor = hwMap.get(DcMotor.class, "wobbleMotor");

        launchServo = hwMap.get(Servo.class, "launchServo");

        liftServo = hwMap.get(Servo.class, "liftServo");
        leftGrabberServo = hwMap.get(Servo.class, "leftGrabberServo");
        rightGrabberServo = hwMap.get(Servo.class, "rightGrabberServo");

//         Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        leftLauncherMotor  = hwMap.get(DcMotor.class, "leftLauncherMotor");
        rightLauncherMotor = hwMap.get(DcMotor.class, "rightLauncherMotor");
        scoopMotor = hwMap.get(DcMotor.class, "scoopMotor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        frontDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
        frontDistanceSensorTwo = hwMap.get(Rev2mDistanceSensor.class, "frontDistanceSensorTwo");
        rearDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "rearDistanceSensor");
        rearDistanceSensorTwo = hwMap.get(Rev2mDistanceSensor.class, "rearDistanceSensorTwo");


        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleServo.setPosition(WOBBLE_SERVO_CLOSED);
        wobbleServoTwoProng.setPosition(WOBBLE_SERVO_TWO_PRONG_CLOSED);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftRearDrive.setPower(0);
//        rightRearDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void teleopInit(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        wobbleServoTwoProng = hwMap.get(Servo.class,"wobbleServoTwoProng");
        wobbleMotor = hwMap.get(DcMotor.class, "wobbleMotor");


        launchServo = hwMap.get(Servo.class, "launchServo");

        liftServo = hwMap.get(Servo.class, "liftServo");

        leftGrabberServo = hwMap.get(Servo.class, "leftGrabberServo");
        rightGrabberServo = hwMap.get(Servo.class, "rightGrabberServo");

//         Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        leftLauncherMotor  = hwMap.get(DcMotor.class, "leftLauncherMotor");
        rightLauncherMotor = hwMap.get(DcMotor.class, "rightLauncherMotor");
        scoopMotor = hwMap.get(DcMotor.class, "scoopMotor");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
        frontDistanceSensorTwo = hwMap.get(Rev2mDistanceSensor.class, "frontDistanceSensorTwo");
        rearDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "rearDistanceSensor");
        rearDistanceSensorTwo = hwMap.get(Rev2mDistanceSensor.class, "rearDistanceSensorTwo");

        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

//         Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//         Set all motors to run without encoders.
//         May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

/*    double getBatteryVoltage() {double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {result = Math.min(result, voltage);}
        }
        return result;
    }*/

    public double getDistanceToWall(Rev2mDistanceSensor sensor, Rev2mDistanceSensor sensorTwo, double defaultDistance) {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        double distanceTwo = sensorTwo.getDistance(DistanceUnit.INCH);
        if (distance > 40) {
            if (distanceTwo > 40) {
                return defaultDistance;
            }
            return distanceTwo;
        }
        return distance;
    }

    public double getDistanceToWall(Rev2mDistanceSensor sensor, double defaultDistance) {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        if (distance > 40)
            return defaultDistance;
        return distance;
    }

    public void resetEncoder () {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setupDriveTrain () {
        resetEncoder();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runUsingEncoder () {
        resetEncoder();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop () {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

}
