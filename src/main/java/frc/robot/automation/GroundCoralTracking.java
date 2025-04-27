package frc.robot.automation;

import static frc.robot.Constants.ObjectAimingConstants.*;

import frc.robot.localization.Localization;

public class GroundCoralTracking {
    private final Localization localization;
    private final ButtonBoardHandler buttonBoardHandler;

    public GroundCoralTracking(Localization localization, ButtonBoardHandler buttonBoardHandler) {
        this.localization = localization;
        this.buttonBoardHandler = buttonBoardHandler;
    }

    public boolean coralBlockingAlignmentOnFarReef() {
        if (buttonBoardHandler.backReefSegmentsSelected()) {
            var coralTargets = localization.getTrackedObjectsFromCamera(kGroundCoralTrackingCameraName);

            coralTargets.removeIf((target) -> !target.isCoral());
            /* Remove coral if it is too high */
            coralTargets.removeIf((target) -> target.getPhotonVisionData().getPitch() > 0);

            if (!coralTargets.isEmpty())
                return true;
        }

        return false;
    }
}
