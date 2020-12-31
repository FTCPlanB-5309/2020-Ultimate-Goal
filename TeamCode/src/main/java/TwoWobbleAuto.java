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

        //Retrieve second wobble, then place in A
        if (target == WobbleTarget.A) {
            drive.backward(0.8, 55);
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(1000);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);

            //Second wobble
//            strafe.right(0.5,6);
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
//            strafe.right(0.5, 48 - (int)robot.getDistanceToWall(10));
            //left = 15, rear = 20
//            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
        }
        else if (target == WobbleTarget.B) {
            drive.backward(0.8, 75);
            gyroTurn.absolute(0);
            strafe.left(0.5, ((int)robot.getDistanceToWall(robot.leftDistanceSensor, robot.leftDistanceSensorTwo,
                    15) - 18));
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(1000);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
        }
        else { //WobbleTarget.C
            drive.backward(0.8, 50);
            gyroTurn.absolute(5);
            drive.backward(0.8, 46);
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            Thread.sleep(1000);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
            gyroTurn.absolute(0);

            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            drive.forward(0.8, 41);
            gyroTurn.absolute(0);
            drive.forward(0.8, 55);
            drive.forward(0.6,(int)robot.getDistanceToWall(robot.frontDistanceSensor,
                    16) - 16);

            gyroTurn.absolute(-90);
            drive.forward(0.5,24 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            gyroTurn.absolute(-5);
//            drive.backward(0.7,94);
//            gyroTurn.absolute(100);
//            drive.forward(0.6,6);
        }
    }

    double getBatteryVoltage() {double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {result = Math.min(result, voltage);}
        }
        return result;
    }
}
