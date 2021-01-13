import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "One-Wobble Autonomous")

public class AutonomousProgram extends LinearOpMode {
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
        Thread.sleep(1000);

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);
        drive.backward(0.5, 60);



        gyroTurn.absolute(0);
        telemetry.addData("Wall Distance", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Wall Distance Two", robot.leftDistanceSensorTwo.getDistance(DistanceUnit.INCH));
        telemetry.update();

        strafe.left(0.5, ((int)robot.getDistanceToWall(robot.leftDistanceSensor, robot.leftDistanceSensorTwo,
                15) - 20));
        gyroTurn.absolute(0);
//        shooter.setLaunchAngle(robot.LAUNCHER_HIGH_ANGLE);
        robot.liftServo.setPosition(robot.LAUNCHER_HIGH_ANGLE);
        shooter.wheelsOn();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.wheelsOff();

        //driveTo (either drive or strafe, or both)
//        drive.backward(0.25, 10);
//        wobble.moveArm(robot.WOBBLE_ARM_DOWN);
//        wobble.release();
//        wobble.moveArm(robot.WOBBLE_ARM_UP);
        wobble.place1stWobble(target);

        //driveTo (drive, turn, drive forward if needed)
        wobble.moveArm(robot.WOBBLE_ARM_DOWN);
        Thread.sleep(1000);
        wobble.release();
        wobble.moveArm(robot.WOBBLE_ARM_UP);
        Thread.sleep(500);
        //gyroTurn.absolute(90);
            double distancetowall;
            switch (target){
                case A:
                    gyroTurn.absolute(0);
                    distancetowall = robot.getDistanceToWall(robot.leftDistanceSensor, robot.leftDistanceSensorTwo,
                            16);
                    strafe.right(0.5, (int)distancetowall + 6);
                    drive.backward(0.5, 12);
                    break;
                case B:
                    drive.forward(0.5, 10);
                    break;
                case C:
                    gyroTurn.absolute(0);
                    drive.forward(0.5, 33);
                    break;
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
