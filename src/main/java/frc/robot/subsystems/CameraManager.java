package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraManager extends SubsystemBase{
    private final CvSink cvSink;
    private final CvSource cvSource;

    public CameraManager() {
        CameraServer.startAutomaticCapture();
        this.cvSink = CameraServer.getVideo();
        this.cvSource = CameraServer.putVideo("Blur", 640, 480);
    }
}
