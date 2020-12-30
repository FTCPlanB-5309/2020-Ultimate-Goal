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
        telemetry.addData("position", target);
        telemetry.update();

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);

//        strafe.left(0.5, ((int)robot.getDistanceToWall(15) - 20));
//        wobble.place1stWobble(target);
//        wobble.moveArm(robot.WOBBLE_ARM_DOWN);
//        wobble.release();
//        wobble.moveArm(robot.WOBBLE_ARM_UP);

        //Retrieve second wobble, then place in A
        if (target == WobbleTarget.A) {
            drive.backward(0.8, 55);
//            telemetry.addData("Wall Distance", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Wall Distance Two", robot.leftDistanceSensorTwo.getDistance(DistanceUnit.INCH));
//            telemetry.update();
            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);

            //Second wobble
//            strafe.right(0.5,6);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            drive.forward(0.8, 46);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            gyroTurn.absolute(-90);
            drive.forward(0.5,20 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            gyroTurn.absolute(165);
//            strafe.right(0.5, 48 - (int)robot.getDistanceToWall(10));
            //left = 15, rear = 20
//            wobble.moveArm(robot.WOBBLE_ARM_DOWN);
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
