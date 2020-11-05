import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous")

public class AutonomousProgram extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
    }
}
