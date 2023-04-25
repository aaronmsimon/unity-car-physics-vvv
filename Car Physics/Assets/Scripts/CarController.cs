using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    public Transform[] wheelTransforms; // Array of wheel transforms
    public float motorForce = 1000f; // The force applied to the car for motor (forward/reverse) movement
    public float steeringForce = 100f; // The force applied to the car for steering
    public float maxSteerAngle = 45f; // The maximum angle for steering
    public float drag = 1f; // The air drag on the car
    public float maxSpeed = 10f; // The maximum speed of the car
    public float suspensionForce = 5000f; // The force applied to simulate suspension
    public float suspensionHeight = 0.5f; // The height of the suspension from the wheel transform's position
    public float suspensionDamping = 100f; // The damping factor for the suspension
    public LayerMask groundLayer; // The layer mask for the ground

    private float currentSpeed; // The current speed of the car
    private Rigidbody rb; // Reference to the rigidbody component

    private PlayerControls playerControls;
    private float accelInput;
    private float steerInput;

    private void Awake() {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = Vector3.zero;
        playerControls = new PlayerControls();
    }

    private void OnEnable() {
        playerControls.Enable();
    }

    private void OnDisable() {
        playerControls.Disable();
    }

    private void Update() {
        accelInput = playerControls.Driving.Acceleration.ReadValue<float>();
        steerInput = playerControls.Driving.Steering.ReadValue<float>();
    }

    void FixedUpdate()
    {
        // Get input for motor and steering
        float motor = motorForce * accelInput;
        float steering = maxSteerAngle * steerInput;

        // Apply motor force to the car's rigidbody
        rb.AddForce(transform.forward * motor);

        // Apply steering force to the car's rigidbody
        rb.AddTorque(transform.up * steering);

        // Calculate current speed of the car
        currentSpeed = rb.velocity.magnitude;

        // Apply air drag to the car's rigidbody
        rb.AddForce(-rb.velocity.normalized * drag * currentSpeed);

        // Limit the car's speed to the maximum speed
        if (currentSpeed > maxSpeed)
        {
            rb.velocity = rb.velocity.normalized * maxSpeed;
        }

        // Update suspension for each wheel
        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            Transform wheelTransform = wheelTransforms[i];
            Vector3 wheelPosition = wheelTransform.position;

            // Cast a ray from the wheel transform's position towards the ground to simulate suspension
            RaycastHit hit;
            if (Physics.Raycast(wheelPosition, -wheelTransform.up, out hit, suspensionHeight, groundLayer))
            {
                // Calculate suspension force based on the raycast hit distance
                float suspensionCompression = suspensionHeight - hit.distance;
                Vector3 suspensionForceVector = -wheelTransform.up * suspensionForce * suspensionCompression;

                // Calculate damping force based on the wheel's velocity
                Vector3 dampingForceVector = -wheelTransform.up * suspensionDamping * rb.GetPointVelocity(wheelPosition).y;

                // Apply suspension and damping forces to the car's rigidbody
                rb.AddForceAtPosition(suspensionForceVector, wheelPosition);
                rb.AddForceAtPosition(dampingForceVector, wheelPosition);
                Debug.Log(hit.distance);
            }
        }
    }
}
