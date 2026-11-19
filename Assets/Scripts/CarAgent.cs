using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class CarAgent : Agent
{
    [System.Serializable]
    public class TrackConfig
    {
        public string trackName;
        public Transform spawnPoint;
        public GameObject checkpointGroup;
    }

    public Car_Controller carController;
    public Rigidbody carRb;

    [Header("Tracks")]
    public TrackConfig[] tracks;

    [Header("Episode Settings")]
    public float maxEpisodeTime = 25f;

    [Header("Spawn Randomization")]
    public float spawnPositionRandomRange = 0.75f;
    public float spawnYawRandomRange = 10f;

    [Header("Reward Settings")]
    public float stepPenalty = -0.001f;
    public float forwardRewardMultiplier = 0.0015f;
    public float idlePenalty = -0.002f;
    public float reversePenalty = -0.004f;
    public float sideSlipPenaltyMultiplier = -0.0002f;
    public float wallPenalty = -1.0f;
    public float checkpointReward = 1.0f;
    public float timeoutPenalty = -0.5f;
    public float finishAllCheckpointsReward = 3.0f;
    public float steeringWithoutThrottlePenalty = -0.001f;

    [Header("Target Progress Reward")]
    public float progressRewardMultiplier = 0.02f;
    public float wrongDirectionPenaltyMultiplier = 0.01f;

    [Header("Raycast Settings")]
    public float rayDistance = 8f;
    public LayerMask wallLayer;

    [Header("Debug")]
    public bool enableDebug = false;
    public bool drawTargetLine = true;
    public bool logRaycasts = false;
    public bool showOnScreenDebug = true;

    private Vector3 startPos;
    private Quaternion startRot;
    private float episodeTimer = 0f;

    private int currentTrackIndex = 0;
    private int totalCheckpoints = 0;
    private int passedCheckpoints = 0;
    private Checkpoint[] activeCheckpoints;

    private float previousDistanceToNextCheckpoint = -1f;

    private float debugFrontRay;
    private float debugLeftRay;
    private float debugRightRay;
    private float debugFrontLeftRay;
    private float debugFrontRightRay;
    private float debugDistanceToCheckpoint;
    private Vector3 debugTargetPos;
    private Vector3 debugLocalTargetDir;
    private string debugTrackName = "";

    public override void Initialize()
    {
        if (carController == null)
            carController = GetComponent<Car_Controller>();

        if (carRb == null)
            carRb = GetComponent<Rigidbody>();

        startPos = transform.position;
        startRot = transform.rotation;

        if (carController != null)
            carController.UseExternalInputs = true;

        if (tracks == null || tracks.Length == 0)
        {
            Debug.LogError("No tracks assigned in CarAgent!");
        }
    }

    public override void OnEpisodeBegin()
    {
        episodeTimer = 0f;
        passedCheckpoints = 0;
        previousDistanceToNextCheckpoint = -1f;

        ChooseRandomTrack();
        ActivateOnlyCurrentTrack();

        if (carRb != null)
        {
            carRb.linearVelocity = Vector3.zero;
            carRb.angularVelocity = Vector3.zero;
        }

        Transform spawn = tracks[currentTrackIndex].spawnPoint;

        if (spawn != null)
        {
            Vector3 randomOffset = new Vector3(
                Random.Range(-spawnPositionRandomRange, spawnPositionRandomRange),
                0f,
                Random.Range(-spawnPositionRandomRange, spawnPositionRandomRange)
            );

            transform.position = spawn.position + randomOffset;
            transform.rotation = spawn.rotation * Quaternion.Euler(
                0f,
                Random.Range(-spawnYawRandomRange, spawnYawRandomRange),
                0f
            );
        }
        else
        {
            transform.position = startPos;
            transform.rotation = startRot;
        }

        if (carRb != null)
        {
            carRb.Sleep();
            carRb.WakeUp();
        }

        if (carController != null)
            carController.SetInputs(0f, 0f, false, false);

        ResetActiveCheckpoints();
        previousDistanceToNextCheckpoint = GetDistanceToNextCheckpoint();

        debugTrackName = tracks != null && tracks.Length > 0 ? tracks[currentTrackIndex].trackName : "Unknown";

        if (enableDebug)
        {
            Debug.Log($"EPISODE BEGIN | Track: {debugTrackName} | Total checkpoints: {totalCheckpoints}");
        }
    }

    private void ChooseRandomTrack()
    {
        if (tracks == null || tracks.Length == 0)
            return;

        currentTrackIndex = Random.Range(0, tracks.Length);
    }

    private void ActivateOnlyCurrentTrack()
    {
        if (tracks == null || tracks.Length == 0)
        {
            activeCheckpoints = new Checkpoint[0];
            totalCheckpoints = 0;
            return;
        }

        for (int i = 0; i < tracks.Length; i++)
        {
            if (tracks[i].checkpointGroup != null)
            {
                tracks[i].checkpointGroup.SetActive(i == currentTrackIndex);
            }
        }

        if (tracks[currentTrackIndex].checkpointGroup != null)
        {
            activeCheckpoints = tracks[currentTrackIndex].checkpointGroup.GetComponentsInChildren<Checkpoint>(true);
            totalCheckpoints = activeCheckpoints.Length;
        }
        else
        {
            activeCheckpoints = new Checkpoint[0];
            totalCheckpoints = 0;
        }
    }

    private void ResetActiveCheckpoints()
    {
        if (activeCheckpoints == null) return;

        foreach (Checkpoint cp in activeCheckpoints)
        {
            if (cp != null)
                cp.passed = false;
        }
    }

    private Checkpoint GetNextCheckpoint()
    {
        if (activeCheckpoints == null) return null;

        for (int i = 0; i < activeCheckpoints.Length; i++)
        {
            if (activeCheckpoints[i] != null && !activeCheckpoints[i].passed)
                return activeCheckpoints[i];
        }

        return null;
    }

    private float GetDistanceToNextCheckpoint()
    {
        Checkpoint next = GetNextCheckpoint();
        if (next == null) return 0f;

        return Vector3.Distance(transform.position, next.transform.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity = transform.InverseTransformDirection(carRb.linearVelocity);

        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.z);
        sensor.AddObservation(carController != null ? carController.Car_Speed_KPH / 100f : 0f);

        Checkpoint nextCheckpoint = GetNextCheckpoint();

        if (nextCheckpoint != null)
        {
            Vector3 localTarget = transform.InverseTransformPoint(nextCheckpoint.transform.position);
            Vector3 localTargetDir = localTarget.normalized;
            float distanceToTarget = Mathf.Clamp(localTarget.magnitude / 30f, 0f, 1f);

            sensor.AddObservation(localTargetDir.x);
            sensor.AddObservation(localTargetDir.z);
            sensor.AddObservation(distanceToTarget);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        sensor.AddObservation(GetRayDistanceNormalized(0f));
        sensor.AddObservation(GetRayDistanceNormalized(20f));
        sensor.AddObservation(GetRayDistanceNormalized(45f));
        sensor.AddObservation(GetRayDistanceNormalized(70f));
        sensor.AddObservation(GetRayDistanceNormalized(-20f));
        sensor.AddObservation(GetRayDistanceNormalized(-45f));
        sensor.AddObservation(GetRayDistanceNormalized(-70f));
        sensor.AddObservation(GetRayDistanceNormalized(90f));
        sensor.AddObservation(GetRayDistanceNormalized(-90f));
    }

    private float GetRayDistanceNormalized(float angleDegrees)
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Vector3 direction = Quaternion.AngleAxis(angleDegrees, transform.up) * transform.forward;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, rayDistance, wallLayer))
        {
            return hit.distance / rayDistance;
        }

        return 1f;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeTimer += Time.fixedDeltaTime;

        if (episodeTimer >= maxEpisodeTime)
        {
            if (enableDebug)
                Debug.Log("EPISODE END: TIMEOUT");

            Academy.Instance.StatsRecorder.Add("Car/EpisodeEndedByTimeout", 1);
            AddReward(timeoutPenalty);
            EndEpisode();
            return;
        }

        float steer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        float rawThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float throttle = 0f;

        if (rawThrottle > 0.15f) throttle = 1f;
        else if (rawThrottle < -0.15f) throttle = -1f;

        bool brake = actions.ContinuousActions[2] > 0f;

        if (carController != null)
            carController.SetInputs(steer, throttle, brake, false);

        Vector3 localVelocity = transform.InverseTransformDirection(carRb.linearVelocity);

        AddReward(stepPenalty);

        if (localVelocity.z > 0.15f)
        {
            AddReward(localVelocity.z * forwardRewardMultiplier);
        }
        else if (localVelocity.z < -0.15f)
        {
            AddReward(reversePenalty);
        }
        else
        {
            AddReward(idlePenalty);
        }

        AddReward(Mathf.Abs(localVelocity.x) * sideSlipPenaltyMultiplier);

        if (Mathf.Abs(steer) > 0.8f && Mathf.Abs(throttle) < 0.2f)
        {
            AddReward(steeringWithoutThrottlePenalty);
        }

        float currentDistance = GetDistanceToNextCheckpoint();

        if (previousDistanceToNextCheckpoint > 0f && currentDistance > 0f)
        {
            float delta = previousDistanceToNextCheckpoint - currentDistance;

            if (delta > 0f)
                AddReward(delta * progressRewardMultiplier);
            else
                AddReward(delta * wrongDirectionPenaltyMultiplier);
        }

        previousDistanceToNextCheckpoint = currentDistance;

        float rayFront = GetRayDistanceNormalized(0f);
        float rayLeft20 = GetRayDistanceNormalized(-20f);
        float rayRight20 = GetRayDistanceNormalized(20f);
        float rayLeft45 = GetRayDistanceNormalized(-45f);
        float rayRight45 = GetRayDistanceNormalized(45f);

        if (rayFront < 0.2f && throttle > 0.5f)
        {
            AddReward(-0.01f);
        }

        if (rayRight45 > rayLeft45 + 0.1f && steer > 0.15f)
        {
            AddReward(0.0015f);
        }

        if (rayLeft45 > rayRight45 + 0.1f && steer < -0.15f)
        {
            AddReward(0.0015f);
        }

        UpdateDebugValues(rayFront);

        Academy.Instance.StatsRecorder.Add("Car/SpeedKPH", carController != null ? carController.Car_Speed_KPH : 0f);
        Academy.Instance.StatsRecorder.Add("Car/ForwardVelocity", localVelocity.z);
        Academy.Instance.StatsRecorder.Add("Car/DistanceToCheckpoint", currentDistance);
        Academy.Instance.StatsRecorder.Add("Car/PassedCheckpoints", passedCheckpoints);
        Academy.Instance.StatsRecorder.Add("Car/ProgressRatio", totalCheckpoints > 0 ? (float)passedCheckpoints / totalCheckpoints : 0f);
        Academy.Instance.StatsRecorder.Add("Car/CurrentTrackIndex", currentTrackIndex);
        Academy.Instance.StatsRecorder.Add("Car/FrontRay", rayFront);
        Academy.Instance.StatsRecorder.Add("Car/Left45Ray", rayLeft45);
        Academy.Instance.StatsRecorder.Add("Car/Right45Ray", rayRight45);
    }

    private void UpdateDebugValues(float rayFront)
    {
        if (!enableDebug) return;

        debugFrontRay = rayFront;
        debugLeftRay = GetRayDistanceNormalized(-90f);
        debugRightRay = GetRayDistanceNormalized(90f);
        debugFrontLeftRay = GetRayDistanceNormalized(-45f);
        debugFrontRightRay = GetRayDistanceNormalized(45f);
        debugDistanceToCheckpoint = GetDistanceToNextCheckpoint();

        Checkpoint next = GetNextCheckpoint();
        if (next != null)
        {
            debugTargetPos = next.transform.position;

            Vector3 localTarget = transform.InverseTransformPoint(next.transform.position);
            debugLocalTargetDir = localTarget.normalized;
        }
        else
        {
            debugTargetPos = transform.position;
            debugLocalTargetDir = Vector3.zero;
        }

        if (logRaycasts)
        {
            Debug.Log(
                $"DEBUG | Track: {debugTrackName} | CP: {passedCheckpoints}/{totalCheckpoints} | " +
                $"Dist: {debugDistanceToCheckpoint:F2} | " +
                $"TargetLocalDir X:{debugLocalTargetDir.x:F2} Z:{debugLocalTargetDir.z:F2} | " +
                $"Rays -> Front:{debugFrontRay:F2} FL:{debugFrontLeftRay:F2} FR:{debugFrontRightRay:F2} L:{debugLeftRay:F2} R:{debugRightRay:F2}"
            );
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;
        actions[0] = Input.GetAxis("Horizontal");
        actions[1] = Input.GetAxis("Vertical");
        actions[2] = Input.GetKey(KeyCode.Space) ? 1f : 0f;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            if (enableDebug)
                Debug.Log("EPISODE END: HIT WALL -> " + collision.gameObject.name);

            Academy.Instance.StatsRecorder.Add("Car/EpisodeEndedByWall", 1);
            AddReward(wallPenalty);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("Checkpoint"))
            return;

        Checkpoint cp = other.GetComponent<Checkpoint>();

        if (cp != null && !cp.passed)
        {
            cp.passed = true;
            passedCheckpoints++;

            if (enableDebug)
            {
                Debug.Log(
                    $"CHECKPOINT HIT: {other.name} | Passed: {passedCheckpoints}/{totalCheckpoints} | Track: {debugTrackName}"
                );
            }

            Academy.Instance.StatsRecorder.Add("Car/CheckpointHit", 1);
            AddReward(checkpointReward);

            episodeTimer = 0f;
            previousDistanceToNextCheckpoint = GetDistanceToNextCheckpoint();

            if (passedCheckpoints >= totalCheckpoints && totalCheckpoints > 0)
            {
                if (enableDebug)
                    Debug.Log("EPISODE END: ALL CHECKPOINTS PASSED");

                Academy.Instance.StatsRecorder.Add("Car/EpisodeEndedBySuccess", 1);
                AddReward(finishAllCheckpointsReward);
                EndEpisode();
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;

        DrawRayGizmo(origin, 0f, Color.blue);
        DrawRayGizmo(origin, 20f, Color.cyan);
        DrawRayGizmo(origin, 45f, Color.cyan);
        DrawRayGizmo(origin, 70f, Color.cyan);
        DrawRayGizmo(origin, -20f, Color.green);
        DrawRayGizmo(origin, -45f, Color.green);
        DrawRayGizmo(origin, -70f, Color.green);
        DrawRayGizmo(origin, 90f, Color.yellow);
        DrawRayGizmo(origin, -90f, Color.yellow);
    }

    private void OnDrawGizmos()
    {
        if (!enableDebug) return;

        Vector3 origin = transform.position + Vector3.up * 0.5f;

        DrawDebugRay(origin, 0f, Color.blue);
        DrawDebugRay(origin, 45f, Color.cyan);
        DrawDebugRay(origin, -45f, Color.cyan);
        DrawDebugRay(origin, 90f, Color.yellow);
        DrawDebugRay(origin, -90f, Color.yellow);

        if (drawTargetLine)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position + Vector3.up * 0.25f, debugTargetPos + Vector3.up * 0.25f);
            Gizmos.DrawSphere(debugTargetPos + Vector3.up * 0.25f, 0.2f);
        }
    }

    private void DrawRayGizmo(Vector3 origin, float angleDegrees, Color color)
    {
        Gizmos.color = color;
        Vector3 direction = Quaternion.AngleAxis(angleDegrees, transform.up) * transform.forward;
        Gizmos.DrawLine(origin, origin + direction * rayDistance);
    }

    private void DrawDebugRay(Vector3 origin, float angleDegrees, Color color)
    {
        Gizmos.color = color;
        Vector3 direction = Quaternion.AngleAxis(angleDegrees, transform.up) * transform.forward;
        Gizmos.DrawLine(origin, origin + direction * rayDistance);
    }

    private void OnGUI()
    {
        if (!enableDebug || !showOnScreenDebug) return;
        if (UnityEngine.Camera.main == null) return;

        Vector3 screenPos = UnityEngine.Camera.main.WorldToScreenPoint(transform.position + Vector3.up * 2f);

        if (screenPos.z < 0f) return;

        string text =
            $"Track: {debugTrackName}\n" +
            $"CP: {passedCheckpoints}/{totalCheckpoints}\n" +
            $"Speed: {(carController != null ? carController.Car_Speed_KPH : 0f):F1}\n" +
            $"Dist: {debugDistanceToCheckpoint:F2}\n" +
            $"Target X: {debugLocalTargetDir.x:F2}\n" +
            $"Target Z: {debugLocalTargetDir.z:F2}\n" +
            $"Front: {debugFrontRay:F2}\n" +
            $"FL: {debugFrontLeftRay:F2}\n" +
            $"FR: {debugFrontRightRay:F2}\n" +
            $"Left: {debugLeftRay:F2}\n" +
            $"Right: {debugRightRay:F2}";

        GUI.Box(new Rect(screenPos.x - 10f, Screen.height - screenPos.y - 10f, 150f, 180f), "");
        GUI.Label(new Rect(screenPos.x, Screen.height - screenPos.y, 140f, 170f), text);
    }
}
