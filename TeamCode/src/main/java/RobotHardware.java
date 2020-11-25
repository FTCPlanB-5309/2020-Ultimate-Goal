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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware
{
    public DcMotor  leftFrontDrive  = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  leftRearDrive  = null;
    public DcMotor  rightRearDrive = null;

    public DcMotor  wobbleMotor = null;

    public DcMotor leftLauncherMotor = null;
    public DcMotor rightLauncherMotor = null;

    public Servo wobbleServo = null;

    public Servo launchServo = null;

    public Servo scoopServo = null;

    public Servo leftGrabberServo = null;
    public Servo rightGrabberServo = null;

    public Servo liftServo = null;

    BNO055IMU imu;

    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor rearDistanceSensor;
    public DistanceSensor frontDistanceSensor;

    HardwareMap hwMap           =  null;

    //Hardware constants

    public static final double TELEOPDEADZONE = 0.05;
    public static final double NOTTURBOFACTOR = 0.5;
    public static final int CLICKS_PER_INCH = 45;
    public static final int STRAFE_CLICKS_PER_INCH = 48;


    public final double HIGH_TURN_POWER = 0.3;
    public final double MEDIUM_TURN_POWER = 0.12;
    public final double LOW_TURN_POWER = 0.04;

    public final double WOBBLE_SERVO_OPEN = 0.24;
    public final double WOBBLE_SERVO_CLOSED = 1.00;
    public final double SCOOPING_POSITION = 1.00;
    public final double DROPPING_POSITION = 0.15;
    public final double DRIVING_POSITION = 0.3;

    public final double LEFT_GRABBER_CLOSED = 0.02;
    public final double RIGHT_GRABBER_CLOSED = 1;
    public final double LEFT_GRABBER_OPEN = 0.35;
    public final double RIGHT_GRABBER_OPEN = 0.66;

    public final double LAUNCHER_FIRE_POSITION = 0;
    public final double LAUNCHER_RESET_POSITION = 1;
    public final double LAUNCHER_HIGH_ANGLE = 0.72;
    public final double LAUNCHER_POWER_ANGLE = 0.65;

    public final double LAUNCHER_LOAD_ANGLE = 0.59;

    //Set real values for these
    public final int WOBBLE_ARM_START = 0;
    public final int WOBBLE_ARM_DOWN = 1200;
    public final int WOBBLE_ARM_UP = 600;

    public RobotHardware(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        wobbleMotor = hwMap.get(DcMotor.class, "wobbleMotor");

        launchServo = hwMap.get(Servo.class, "launchServo");

        scoopServo = hwMap.get(Servo.class, "scoopServo");
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


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




//        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistanceSensor");
//        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistanceSensor");
//        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistanceSensor");
//        rearDistanceSensor = hwMap.get(DistanceSensor.class, "rearDistanceSensor");

        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistanceSensor");
        Rev2mDistanceSensor frontSensorTimeOfFlight = (Rev2mDistanceSensor)frontDistanceSensor;
        rearDistanceSensor = hwMap.get(DistanceSensor.class, "rearDistanceSensor");
        Rev2mDistanceSensor rearSensorTimeOfFlight = (Rev2mDistanceSensor)rearDistanceSensor;
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistanceSensor");
        Rev2mDistanceSensor leftSensorTimeOfFlight = (Rev2mDistanceSensor)leftDistanceSensor;
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistanceSensor");
        Rev2mDistanceSensor rightSensorTimeOfFlight = (Rev2mDistanceSensor)rightDistanceSensor;

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleServo.setPosition(WOBBLE_SERVO_CLOSED);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    }

    public void teleopInit(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        wobbleMotor = hwMap.get(DcMotor.class, "wobbleMotor");

        scoopServo = hwMap.get(Servo.class, "scoopServo");

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

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistanceSensor");
        Rev2mDistanceSensor frontSensorTimeOfFlight = (Rev2mDistanceSensor)frontDistanceSensor;
        rearDistanceSensor = hwMap.get(DistanceSensor.class, "rearDistanceSensor");
        Rev2mDistanceSensor rearSensorTimeOfFlight = (Rev2mDistanceSensor)rearDistanceSensor;
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistanceSensor");
        Rev2mDistanceSensor leftSensorTimeOfFlight = (Rev2mDistanceSensor)leftDistanceSensor;
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistanceSensor");
        Rev2mDistanceSensor rightSensorTimeOfFlight = (Rev2mDistanceSensor)rightDistanceSensor;

//         Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//         Set all motors to run without encoders.
//         May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
