import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

public class WobbleTargetFinder {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public WobbleTargetFinder(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public WobbleTarget search() throws InterruptedException {
        if (robot.tfod != null)
            robot.tfod.activate();
        if (linearOpMode.opModeIsActive()) {
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel() == "Quad")
                            return WobbleTarget.C;
                        else if (recognition.getLabel() == "Single")
                            return WobbleTarget.B;
                        else
                            return WobbleTarget.A;
                    }
                    telemetry.update();
                }
            }
        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }

        return WobbleTarget.A;
    }
}
