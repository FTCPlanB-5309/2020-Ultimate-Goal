import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Shooter (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void setLaunchAngle(double launchAngle) {
        robot.liftServo.setPosition(launchAngle);
    }

    public void triggerFire() {
        robot.launchServo.setPosition(robot.LAUNCHER_FIRE_POSITION);
    }

    public void triggerReset() {
        robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);
    }

    public void wheelsOn() {
        robot.leftLauncherMotor.setPower(1);
        robot.rightLauncherMotor.setPower(1);
    }

    public  void wheelsOff() {
        robot.leftLauncherMotor.setPower(0);
        robot.rightLauncherMotor.setPower(0);
    }

}
