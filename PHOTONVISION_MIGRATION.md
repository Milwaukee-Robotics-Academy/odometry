PhotonVision migration notes

What changed
------------
- The project previously used the deprecated PhotonVision API that included a PhotonPoseEstimator constructor accepting a PoseStrategy and an update(PhotonPipelineResult) method.
- These APIs were replaced in newer PhotonVision versions. To quiet deprecation/removal warnings and migrate to the supported API, `src/main/java/frc/robot/subsystems/swervedrive/Vision.java` was updated to:
  - Use the new PhotonPoseEstimator constructor that accepts an AprilTagFieldLayout and a Transform3d (robot->camera transform).
  - Replace `poseEstimator.update(result)` calls with the new estimation helpers: `estimateCoprocMultiTagPose(result)` (preferred) and `estimateLowestAmbiguityPose(result)` as a fallback.

Why this was done
------------------
- PhotonVision removed/changed the older pose-estimation APIs; the project was producing "deprecated and marked for removal" compile warnings.
- The migration keeps the original behavior intent (prefer coprocessor multi-tag estimation, fall back to lower-ambiguity estimates) while using supported APIs.

What to watch for / future work
-------------------------------
- PhotonVision may add or rename estimation helpers in future releases. If you update the vendordep (`vendordeps/photonlib.json`) to a newer PhotonVision release, re-run the build and fix any compile errors by checking the PhotonVision javadocs (https://javadocs.photonvision.org/).
- Consider adding unit or integration tests for vision estimation (simulation-based) to validate future library upgrades.
- If you want different fallback behavior (e.g., constrained solvePNP or Rio-side multi-tag estimation), update `Vision.java`'s estimator selection logic accordingly.

How to reproduce
----------------
- Build:

```powershell
./gradlew.bat clean build --console=plain
```

- If you see deprecation/removal warnings after updating PhotonVision, check `Vision.java` and the PhotonVision javadocs for the new equivalents of the removed APIs.

Contact / notes
----------------
- PhotonVision docs: https://docs.photonvision.org/
- PhotonVision javadocs: https://javadocs.photonvision.org/

