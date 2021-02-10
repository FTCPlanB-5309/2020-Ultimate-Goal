import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RingArm {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    final int CLICKS_PER_REVOLUTION = 288;

    public RingArm (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void moveToPosition (int clicks) {
        if (clicks > robot.scoopMotor.getCurrentPosition())
            robot.scoopMotor.setPower(1);
        else
            robot.scoopMotor.setPower(-1);
        robot.scoopMotor.setTargetPosition(clicks);
        robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.scoopMotor.isBusy()) {
            telemetry.addData("Arm Clicks", robot.scoopMotor.getCurrentPosition());
            telemetry.addData("Target Position", clicks);
            telemetry.update();
            Thread.yield();
        }
        robot.scoopMotor.setPower(0);
    }
}
