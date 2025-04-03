package frc.robot.automation;

import static frc.robot.Constants.ObjectAimingConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
            var coralTargets = getLatestCoralTargets();

            if (!coralTargets.isEmpty())
                return true;
        }

        return false;
    }

    private List<PhotonTrackedTarget> getLatestCoralTargets() {
        var results = new ArrayList<>(localization.getObjectTrackingResults(kGroundCoralTrackingCameraName));

        /* Remove all results without targets - exit if no results have targets */
        results.removeIf((result) -> !result.hasTargets());
        if(results.isEmpty())
            return List.of();

        /* Find the most recent result */
        PhotonPipelineResult latestResult = results.get(0);
        for(var result : results) {
            if(result.getTimestampSeconds() > latestResult.getTimestampSeconds())
                latestResult = result;
        }

        /* Remove targets that are not coral  */
        var filteredTargets = latestResult.getTargets();
        filteredTargets.removeIf((target) -> target.getDetectedObjectClassID() != kCoralId);

        return filteredTargets;        
    }
}
