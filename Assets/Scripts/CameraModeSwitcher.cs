using UnityEngine;

public class CameraModeSwitcher : MonoBehaviour
{
    [Header("Targets")]
    public Transform carTarget;

    [Header("Follow Camera")]
    public Vector3 followOffset = new Vector3(0f, 5f, -8f);
    public float followSmooth = 5f;
    public float lookSmooth = 8f;

    [Header("Free Camera")]
    public float freeMoveSpeed = 10f;
    public float fastMoveMultiplier = 2f;
    public float mouseSensitivity = 2f;

    [Header("Mode")]
    public bool followMode = true;
    public KeyCode switchKey = KeyCode.C;

    private float rotX;
    private float rotY;

    void Start()
    {
        Vector3 angles = transform.eulerAngles;
        rotY = angles.y;
        rotX = angles.x;
    }

    void Update()
    {
        if (Input.GetKeyDown(switchKey))
        {
            followMode = !followMode;
        }

        if (!followMode)
        {
            FreeCameraUpdate();
        }
    }

    void LateUpdate()
    {
        if (followMode && carTarget != null)
        {
            FollowCameraUpdate();
        }
    }

    void FollowCameraUpdate()
    {
        Vector3 desiredPosition = carTarget.TransformPoint(followOffset);
        transform.position = Vector3.Lerp(transform.position, desiredPosition, followSmooth * Time.deltaTime);

        Quaternion targetRotation = Quaternion.LookRotation(carTarget.position - transform.position, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, lookSmooth * Time.deltaTime);
    }

    void FreeCameraUpdate()
    {
        if (Input.GetMouseButton(1))
        {
            rotY += Input.GetAxis("Mouse X") * mouseSensitivity;
            rotX -= Input.GetAxis("Mouse Y") * mouseSensitivity;
            rotX = Mathf.Clamp(rotX, -89f, 89f);

            transform.rotation = Quaternion.Euler(rotX, rotY, 0f);
        }

        float speed = freeMoveSpeed;
        if (Input.GetKey(KeyCode.LeftShift))
            speed *= fastMoveMultiplier;

        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.W)) move += transform.forward;
        if (Input.GetKey(KeyCode.S)) move -= transform.forward;
        if (Input.GetKey(KeyCode.A)) move -= transform.right;
        if (Input.GetKey(KeyCode.D)) move += transform.right;
        if (Input.GetKey(KeyCode.E)) move += Vector3.up;
        if (Input.GetKey(KeyCode.Q)) move -= Vector3.up;

        transform.position += move.normalized * speed * Time.deltaTime;
    }
}
