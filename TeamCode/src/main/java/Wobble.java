import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wobble {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Wobble (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void moveArm (int clicks) {
        int target = robot.wobbleMotor.getTargetPosition() + clicks;
        if (target > robot.wobbleMotor.getCurrentPosition())
            robot.wobbleMotor.setPower(1);
        else
            robot.wobbleMotor.setPower(-1);
        robot.wobbleMotor.setTargetPosition(target);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(robot.wobbleMotor.getCurrentPosition() - target) > 2) {
            telemetry.addData("Arm Clicks", robot.wobbleMotor.getCurrentPosition());
            telemetry.addData("Target Position", target);
            telemetry.update();
            Thread.yield();
        }
        robot.wobbleMotor.setPower(0);
    }

    public void grab() {
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_CLOSED);
    }

    public void release() {
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
    }
}
