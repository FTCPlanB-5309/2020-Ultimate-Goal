import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// Purpose: This is an object that can be used to manipulate the wobble attachment during the autonomous program

public class Wobble {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    Drive drive;
    GyroTurn gyroTurn;
    Strafe strafe;

    // Resturctured the Wobble constructor so that the needed objects are created here using the
    // object global variables declared above.  
    public Wobble (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        drive = new Drive(robot, telemetry, linearOpMode);
        gyroTurn = new GyroTurn(robot, telemetry, linearOpMode);
        strafe = new Strafe(robot, telemetry, linearOpMode);
    }

    public void moveArm (int clicks) {
        if (clicks > robot.wobbleMotor.getCurrentPosition())
            robot.wobbleMotor.setPower(1);
        else
            robot.wobbleMotor.setPower(-1);
        robot.wobbleMotor.setTargetPosition(clicks);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(robot.wobbleMotor.getCurrentPosition() - clicks) > 2) {
            telemetry.addData("Arm Clicks", robot.wobbleMotor.getCurrentPosition());
            telemetry.addData("Target Position", clicks);
            telemetry.update();
            Thread.yield();
        }
        robot.wobbleMotor.setPower(0);
    }

    public void grab() throws InterruptedException {
        robot.leftWobbleServo.setPosition(robot.LEFT_WOBBLE_SERVO_CLOSED);
        robot.rightWobbleServo.setPosition(robot.RIGHT_WOBBLE_SERVO_CLOSED);
        Thread.sleep(1000);
    }

    public void release() throws  InterruptedException {
        robot.leftWobbleServo.setPosition(robot.LEFT_WOBBLE_SERVO_OPEN);
        robot.rightWobbleServo.setPosition(robot.RIGHT_WOBBLE_SERVO_OPEN);
        Thread.sleep(1000);
    }

    public void place1stWobble(WobbleTarget wobbleTarget) throws InterruptedException {
        double distancetowall;
        switch (wobbleTarget){
            case A:
                distancetowall = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("Distance to wall", distancetowall);
                telemetry.update();
                Thread.sleep(3000);
                strafe.left(0.5, (int)distancetowall - 6);
                gyroTurn.absolute(0);
                break;
            case B:
                distancetowall = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
                strafe.right(0.5, (int)distancetowall + 6);
                Thread.sleep(100);
                distancetowall = robot.rearDistanceSensor.getDistance(DistanceUnit.INCH);
                drive.backward(0.5, 14);
                break;
            case C:
                distancetowall = robot.rearDistanceSensor.getDistance(DistanceUnit.INCH);
                drive.backward(0.5, (int)distancetowall - 12);
                Thread.sleep(100);
                distancetowall = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
                strafe.left(0.5, (int)distancetowall - 6);
                break;
        }
    }
}
