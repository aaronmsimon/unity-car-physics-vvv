using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tire : MonoBehaviour
{
    [Header("Tire Info")]
    [SerializeField] private Transform carTransform;
    [SerializeField] private bool steering;
    [SerializeField] private float maxSteerAngle;
    [SerializeField] private bool accelerating;

    [Header("Suspension")]
    [SerializeField] private float suspensionRestDist;
    [SerializeField] private float springStrength;
    [SerializeField] private float springDamper;

    [Header("Steering")]
    [SerializeField] [Range(0,1)] private float tireGripFactor;
    [SerializeField] private float tireMass;

    [Header("Acceleration")]
    [SerializeField] private float carTopSpeed;

    [Header("Debug Options")]
    [SerializeField] private bool drawAxes;
    [SerializeField] private bool showGroundCheckRaycast;
    [SerializeField] private bool showSuspensionForce;
    [SerializeField] private bool showSteeringForce;
    [SerializeField] private bool showAccelerationForce;
    [SerializeField] [Range(0,1)] private float forceDistanceScale;

    private bool rayDidHit;
    private RaycastHit tireRay;
    private float axesLength = .25f;

    private Rigidbody carRigidBody;
    private PlayerControls playerControls;

    private float steerAngle;
    private float accelInput;

    private void Awake() {
        carRigidBody = GetComponentInParent<Rigidbody>();
        playerControls = new PlayerControls();
    }

    private void OnEnable() {
        playerControls.Enable();
    }

    private void OnDisable() {
        playerControls.Disable();
    }

    private void Update() {
        // Check if grounded
        GroundRaycast();

        // Get Player Input
        float steerInput = playerControls.Driving.Steering.ReadValue<float>();
        steerAngle = Mathf.Clamp(steerInput * maxSteerAngle, -maxSteerAngle, maxSteerAngle);
        accelInput = playerControls.Driving.Acceleration.ReadValue<float>();

        // Show Ground Raycast
        if (showGroundCheckRaycast) {
            Debug.DrawRay(transform.position, -transform.up * tireRay.distance, Color.yellow);
        }

        // Draw Axes
        if (drawAxes) {
            Debug.DrawLine(transform.position, transform.position + transform.up * axesLength, Color.green);
            Debug.DrawLine(transform.position + transform.right * axesLength, transform.position - transform.right * axesLength, Color.red);
            Debug.DrawLine(transform.position, transform.position + transform.forward * axesLength, Color.blue);
        }
    }

    private void FixedUpdate() {
        if (rayDidHit) {
            // suspension spring force
            ApplySpringForce();
            // steering force
            ApplySteeringForce();
            if (accelerating) {
                // acceleration force
                ApplyAccelerationForce();
            }
        }

        // turn steering tires
        if (steering) {
            transform.rotation = Quaternion.Euler(0f, steerAngle, 0f);
        }
    }

    private void GroundRaycast() {
        rayDidHit = Physics.Raycast(transform.position, -transform.up, out tireRay);
    }

    private void ApplySpringForce() {
        // world-space direction of the spring force.
        Vector3 springDir = transform.up;

        // world-space velocity of this tire
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(transform.position);

        // calculate offset from the raycast
        float offset = suspensionRestDist - tireRay.distance;

        // calculate velocity along the spring direction
        // note that springDir is a unit vector, so this returns the magnitude of tireWorldVel
        // as projected onto springDir
        float vel = Vector3.Dot(springDir, tireWorldVel);

        // calculate the magnitude of the dampened spring force!
        float force = (offset * springStrength) - (vel * springDamper);

        // apply the force at the location of this tire, in the direction
        // of the suspension
        Vector3 suspensionForce = springDir * force;
        carRigidBody.AddForceAtPosition(suspensionForce, transform.position);

        // Show Suspension Force
        if (showSuspensionForce) {
            Debug.DrawRay(transform.position, suspensionForce * forceDistanceScale, Color.green);
        }
    }

    private void ApplySteeringForce() {
        // world-space direction of the spring force.
        Vector3 steeringDir = transform.right;

        // world-space velocity of the suspension
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(transform.position);

        // what is the tire's velocity in the steering direction?
        // note that steeringDir is a unit vector, so this returns the magnitude of tireWorldVel
        // as projected onto steeringDir
        float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);

        // the change in velocity that we're looking for is -steeringVel * gripFactor
        // gripFactor is in range 0-1, 0 means no grip, 1 means full grip
        float desiredVelChange = -steeringVel * tireGripFactor;

        // turn change in velocity into an acceleration (acceleration = change in vel / time)
        // this will produce the acceleration necdessary to change the velocity by desiredVelChange in 1 physics step
        float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

        // Force = Mass * Acceleration, so multiply by the mass of the tire and apply as a force!
        Vector3 steeringForce = steeringDir * tireMass * desiredAccel;
        carRigidBody.AddForceAtPosition(steeringForce, transform.position);

        // Show Steering Force
        if (showSteeringForce) {
            Debug.DrawRay(transform.position, steeringForce, Color.red);
        }
    }

    private void ApplyAccelerationForce() {
        // world-space direction of the acceleration/braking force.
        Vector3 accelDir = transform.forward;

        // acceleration torque
        if (accelInput > 0f) {
            // forward speed of the car (in the direction of driving)
            float carSpeed = Vector3.Dot(carTransform.forward, carRigidBody.velocity);

            // normalized car speed
            float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

            // available torque
            //float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput;
            float availableTorque = accelInput;
            Vector3 accelerationForce = accelDir * availableTorque;

            carRigidBody.AddForceAtPosition(accelerationForce, transform.position);

            // Show Acceleration Force
            if (showAccelerationForce) {
                Debug.DrawRay(transform.position, accelerationForce, Color.blue);
            }
        }
    }
}
