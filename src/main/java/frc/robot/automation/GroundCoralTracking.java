package frc.robot.automation;

import static frc.robot.Constants.ObjectAimingConstants.*;

import frc.robot.localization.Localization;
import frc.robot.utils.FieldUtil;

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

            /* Remove coral if it is too high (prevent seeing coral in L1) and is not close to the reef */
            coralTargets.removeIf(
                (coral) -> coral.getPhotonVisionData().getPitch() > 0 ||
                !FieldUtil.isCloseToReef(coral.getFieldPosition().getTranslation())
            );

            if (!coralTargets.isEmpty())
                return true;
        }

        return false;
    }
}
