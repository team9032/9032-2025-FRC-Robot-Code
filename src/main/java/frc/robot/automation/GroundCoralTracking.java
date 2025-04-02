package frc.robot.automation;

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
            localization.getObjectTrackingResults("etttwtr");

            
        }return false;
    }
}
