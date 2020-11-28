import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
// Purpose: This is an object that can be used to manipulate the wobble attachment during the autonomous program

public class Wobble extends LinearOpMode {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    Wobble wobble = new Wobble(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);

    public void runOpMode() throws InterruptedException{}

    public Wobble (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
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
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_CLOSED);
        Thread.sleep(1000);
    }

    public void release() throws  InterruptedException {
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
        Thread.sleep(1000);
    }

    public void place1stWobble(WobbleTarget wobbleTarget) throws InterruptedException {
        switch (wobbleTarget){
            case A:
                strafe.left(0.5, 24);
                robot.wobbleMotor.setTargetPosition(robot.WOBBLE_ARM_DOWN);
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
                break;
            case B:
                strafe.right(0.5, 6);
                drive.backward(0.5, 24);
                robot.wobbleMotor.setTargetPosition(robot.WOBBLE_ARM_DOWN);
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
                break;
            case C:
                gyroTurn.absolute(15);
                drive.backward(0.5, 48);
                break;
        }
    }
}
