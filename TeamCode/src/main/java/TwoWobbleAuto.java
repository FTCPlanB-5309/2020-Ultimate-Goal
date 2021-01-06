import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Wobble-Only Autonomous")

public class TwoWobbleAuto extends LinearOpMode{
    RobotHardware robot = new RobotHardware();
    Wobble wobble = new Wobble(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    Shooter shooter = new Shooter(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    WobbleTargetFinder wobbleFinder = new WobbleTargetFinder(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.voltage = getBatteryVoltage();
        telemetry.addData("voltage", robot.voltage);
        telemetry.update();
        waitForStart();

        WobbleTarget target = wobbleFinder.search();
//        WobbleTarget target = WobbleTarget.C;
        telemetry.addData("position", target);
        telemetry.update();

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);

        drive.backward(0.8, 60);
        gyroTurn.absolute(10);
        robot.liftServo.setPosition(robot.LAUNCHER_HIGH_ANGLE);
        shootThreeRings();
        gyroTurn.absolute(2);

        //Retrieve second wobble, then place in A
        if (target == WobbleTarget.A) {
            drive.forward(0.8, 5);
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(1000);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);

            //Second wobble
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            drive.forward(0.8, 46);
            gyroTurn.absolute(-90);
            drive.forward(0.5,24 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            gyroTurn.absolute(165);
            drive.forward(0.7,46);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            drive.backward(0.5,5);
            strafe.left(0.5,8);
        }
        else if (target == WobbleTarget.B) {
            drive.backward(0.8, 15);
            gyroTurn.absolute(0);
            strafe.left(0.5, ((int)robot.getDistanceToWall(robot.leftDistanceSensor,
                    robot.leftDistanceSensorTwo,
                    15) - 18));
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(1000);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
        }
        else { //WobbleTarget.C
            drive.backward(0.8, 36);
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(800);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
            gyroTurn.absolute(0);

            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            //Second wobble
            drive.forward(0.8, 85);
            Thread.sleep(500);
            double distance = robot.getDistanceToWall(robot.frontDistanceSensor, 13);
            if (distance > 13)
                drive.forward(0.3, (int)distance - 13);
            else
                drive.backward(0.3, 13 - (int)distance);
            gyroTurn.absolute(-85);
            drive.forward(0.6, 23 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            Thread.sleep(500);
            robot.liftServo.setPosition(robot.RING_SERVO_BARELY_LIFTED);
            gyroTurn.absolute(-5);
            drive.backward(0.8, 87);
            gyroTurn.absolute(134);
            drive.forward(0.5,15);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            Thread.sleep(100);
            drive.backward(0.9,36);

//            drive.forward(0.5,15 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
//            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);

//            drive.forward(0.6,(int)robot.getDistanceToWall(robot.frontDistanceSensor,
//                    16) - 16);
//
//            gyroTurn.absolute(-90);
//            drive.forward(0.5,24 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
//            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
//            gyroTurn.absolute(-5);
//            drive.backward(0.7,94);
//            gyroTurn.absolute(100);
//            drive.forward(0.6,6);
        }

    }

    private void shootThreeRings() throws InterruptedException {
        shooter.wheelsOn();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.wheelsOff();
    }

    double getBatteryVoltage() {double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {result = Math.min(result, voltage);}
        }
        return result;
    }
}
