using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class CarAgent1 : Agent
{
    public Car_Controller carController;
    public Rigidbody carRb;
    public Transform spawnPoint;

    [Header("Episode Settings")]
    public float maxEpisodeTime = 25f;

    [Header("Reward Settings")]
    public float stepPenalty = -0.001f;
    public float forwardRewardMultiplier = 0.003f;
    public float idlePenalty = -0.003f;
    public float reversePenalty = -0.02f;
    public float sideSlipPenaltyMultiplier = -0.0003f;
    public float wallPenalty = -1.0f;
    public float checkpointReward = 1.0f;
    public float timeoutPenalty = -0.5f;
    public float finishAllCheckpointsReward = 3.0f;
    public float steeringWithoutThrottlePenalty = -0.002f;

    [Header("Raycast Settings")]
    public float rayDistance = 8f;
    public LayerMask wallLayer;

    private Vector3 startPos;
    private Quaternion startRot;
    private float episodeTimer = 0f;

    private int totalCheckpoints = 0;
    private int passedCheckpoints = 0;

    public override void Initialize()
    {
        if (carController == null)
            carController = GetComponent<Car_Controller>();

        if (carRb == null)
            carRb = GetComponent<Rigidbody>();

        startPos = transform.position;
        startRot = transform.rotation;

        carController.UseExternalInputs = true;

        Checkpoint[] checkpoints = FindObjectsOfType<Checkpoint>();
        totalCheckpoints = checkpoints.Length;
    }

    public override void OnEpisodeBegin()
    {
        episodeTimer = 0f;
        passedCheckpoints = 0;

        if (carRb != null)
        {
            carRb.linearVelocity = Vector3.zero;
            carRb.angularVelocity = Vector3.zero;
        }

        if (spawnPoint != null)
        {
            transform.position = spawnPoint.position;
            transform.rotation = spawnPoint.rotation;
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

        carController.SetInputs(0f, 0f, false, false);

        Checkpoint[] checkpoints = FindObjectsOfType<Checkpoint>();
        totalCheckpoints = checkpoints.Length;

        foreach (Checkpoint cp in checkpoints)
        {
            cp.passed = false;
        }

        Debug.Log("EPISODE BEGIN");
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity = transform.InverseTransformDirection(carRb.linearVelocity);

        // Базовые наблюдения
        sensor.AddObservation(localVelocity.x); // боковая скорость
        sensor.AddObservation(localVelocity.z); // скорость вперед/назад
        sensor.AddObservation(transform.forward.x);
        sensor.AddObservation(transform.forward.z);
        sensor.AddObservation(carController.Car_Speed_KPH / 100f);

        // Raycast-наблюдения
        sensor.AddObservation(GetRayDistanceNormalized(transform.forward)); // вперед
        sensor.AddObservation(GetRayDistanceNormalized((transform.forward + transform.right * 0.5f).normalized)); // вперед-вправо
        sensor.AddObservation(GetRayDistanceNormalized((transform.forward - transform.right * 0.5f).normalized)); // вперед-влево
        sensor.AddObservation(GetRayDistanceNormalized(transform.right)); // вправо
        sensor.AddObservation(GetRayDistanceNormalized(-transform.right)); // влево
    }

    private float GetRayDistanceNormalized(Vector3 direction)
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;

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
            Debug.Log("EPISODE END: TIMEOUT");
            AddReward(timeoutPenalty);
            EndEpisode();
            return;
        }

        float steer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        float rawThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float throttle = 0f;
        if (rawThrottle > 0.2f) throttle = 1f;
        else if (rawThrottle < -0.2f) throttle = -1f;

        bool brake = actions.ContinuousActions[2] > 0f;

        carController.SetInputs(steer, throttle, brake, false);

        Vector3 localVelocity = transform.InverseTransformDirection(carRb.linearVelocity);

        AddReward(stepPenalty);

        if (localVelocity.z > 0.2f)
        {
            AddReward(localVelocity.z * forwardRewardMultiplier);
        }
        else if (localVelocity.z < -0.1f)
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

        // Бонус/штраф по лучам
        float frontRay = GetRayDistanceNormalized(transform.forward);
        float frontRightRay = GetRayDistanceNormalized((transform.forward + transform.right * 0.5f).normalized);
        float frontLeftRay = GetRayDistanceNormalized((transform.forward - transform.right * 0.5f).normalized);

        // Если впереди совсем близко стена и он всё равно прёт, слегка штрафуем
        if (frontRay < 0.2f && throttle > 0.5f)
        {
            AddReward(-0.01f);
        }

        // Если справа больше места, а он рулит вправо — маленький бонус
        if (frontRightRay > frontLeftRay + 0.1f && steer > 0.2f)
        {
            AddReward(0.002f);
        }

        // Если слева больше места, а он рулит влево — маленький бонус
        if (frontLeftRay > frontRightRay + 0.1f && steer < -0.2f)
        {
            AddReward(0.002f);
        }

        Academy.Instance.StatsRecorder.Add("Car/SpeedKPH", carController.Car_Speed_KPH);
        Academy.Instance.StatsRecorder.Add("Car/ForwardVelocity", localVelocity.z);
        Academy.Instance.StatsRecorder.Add("Car/FrontRay", frontRay);
        Academy.Instance.StatsRecorder.Add("Car/FrontRightRay", frontRightRay);
        Academy.Instance.StatsRecorder.Add("Car/FrontLeftRay", frontLeftRay);
        Academy.Instance.StatsRecorder.Add("Car/PassedCheckpoints", passedCheckpoints);
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
            Debug.Log("EPISODE END: HIT WALL -> " + collision.gameObject.name);
            AddReward(wallPenalty);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Checkpoint"))
        {
            Checkpoint cp = other.GetComponent<Checkpoint>();

            if (cp != null && !cp.passed)
            {
                cp.passed = true;
                passedCheckpoints++;

                Debug.Log("CHECKPOINT HIT: " + other.name + " | Passed: " + passedCheckpoints + "/" + totalCheckpoints);

                AddReward(checkpointReward);

                episodeTimer = 0f;

                if (passedCheckpoints >= totalCheckpoints && totalCheckpoints > 0)
                {
                    Debug.Log("EPISODE END: ALL CHECKPOINTS PASSED");
                    AddReward(finishAllCheckpointsReward);
                    EndEpisode();
                }
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;

        DrawRayGizmo(origin, transform.forward, Color.blue);
        DrawRayGizmo(origin, (transform.forward + transform.right * 0.5f).normalized, Color.cyan);
        DrawRayGizmo(origin, (transform.forward - transform.right * 0.5f).normalized, Color.cyan);
        DrawRayGizmo(origin, transform.right, Color.yellow);
        DrawRayGizmo(origin, -transform.right, Color.yellow);
    }

    private void DrawRayGizmo(Vector3 origin, Vector3 dir, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(origin, origin + dir * rayDistance);
    }
}
