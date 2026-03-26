package frc.robot.util;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class Aimer {

    private final TurnToPoseController turnController = new TurnToPoseController(4, 0, 0);
    private final Drive m_drive;

    private Pose2d target;

    public Aimer(Drive drive) {
        m_drive = drive;
    }

    public void setTarget(Pose2d t) {
        target = t;
        if (target != null) {
            PPHolonomicDriveController.overrideRotationFeedback(this::calculate);
        } else {
            PPHolonomicDriveController.clearRotationFeedbackOverride();
        }
    }

    public boolean hasTarget() {
        return target != null;
    }

    public double calculate() {
        return turnController.calculate(m_drive.getPose(), target);
    }
}
