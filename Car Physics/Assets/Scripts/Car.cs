using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Car : MonoBehaviour
{
    [Header("Tire Info")]
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
    [SerializeField] private float carTorque;
    [SerializeField] private float carTopSpeed;

    [Header("Debug Options")]
    [SerializeField] private bool drawAxes;
    [SerializeField] private bool showGroundCheckRaycast;
    [SerializeField] private bool showSuspensionForce;
    [SerializeField] private bool showSteeringForce;
    [SerializeField] private bool showAccelerationForce;
    [SerializeField] [Range(0,1)] private float forceDistanceScale;

    // Modular variables
    private bool rayDidHit;
    private RaycastHit tireRay;
    private float accelInput;
    private float steerAngle;
    private float axesLength = .25f;

    // Awake variables
    private Transform carTransform;
    private Rigidbody carRigidBody;
    private PlayerControls playerControls;

    private void Awake() {
        carTransform = transform;
        carRigidBody = GetComponent<Rigidbody>();
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
        float steerInput = playerControls.Driving.Steering.ReadValue<float>();
        steerAngle = Mathf.Clamp(steerInput * maxSteerAngle, -maxSteerAngle, maxSteerAngle);

    }

    private void FixedUpdate() {
        Transform tires;
        tires = transform.Find("Tires");

        // Iterate through each child object
        for (int i = 0; i < tires.childCount; i++)
        {
            Transform tire = tires.GetChild(i);
            TireRaycast(tire);

            if (tire.name.Substring(0,1) == "F") {
                tire.Rotate(tire.up, steerAngle - tire.localEulerAngles.y, Space.Self);
            }

            if (rayDidHit) {
                ApplySpringForce(tire);
                ApplySteeringForce(tire);
                ApplyAccelerationForce(tire);
            }

            // Draw Axes
            if (drawAxes) {
                Debug.DrawLine(tire.position, tire.position + tire.up * axesLength, Color.green);
                Debug.DrawLine(tire.position + tire.right * axesLength, tire.position - tire.right * axesLength, Color.red);
                Debug.DrawLine(tire.position, tire.position + tire.forward * axesLength, Color.blue);
            }
        }
        
    }

    private void TireRaycast(Transform tireTransform) {
        rayDidHit = Physics.Raycast(tireTransform.position, -tireTransform.up, out tireRay);

        // Show Ground Raycast
        if (showGroundCheckRaycast) {
            Debug.DrawRay(tireTransform.position, -tireTransform.up * tireRay.distance, Color.yellow);
        }
    }

    private void ApplySpringForce(Transform tireTransform) {
        // world-space direction of the spring force.
        Vector3 springDir = tireTransform.up;

        // world-space velocity of this tire
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform.position);

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
        carRigidBody.AddForceAtPosition(suspensionForce, tireTransform.position);

        // Show Suspension Force
        if (showSuspensionForce) {
            Debug.DrawLine(tireTransform.position, tireTransform.position + suspensionForce * forceDistanceScale, Color.green);
        }
    }

    private void ApplySteeringForce(Transform tireTransform) {
        // world-space direction of the spring force.
        Vector3 steeringDir = tireTransform.right;

        // world-space velocity of the suspension
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform.position);

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
            Debug.DrawRay(tireTransform.position, steeringForce, Color.red);
        }
    }

    private void ApplyAccelerationForce(Transform tireTransform) {
        // world-space direction of the acceleration/braking force.
        Vector3 accelDir = tireTransform.forward;

        // acceleration torque
        if (accelInput > 0f) {
            // forward speed of the car (in the direction of driving)
            float carSpeed = Vector3.Dot(carTransform.forward, carRigidBody.velocity);

            // normalized car speed
            float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

            // available torque
            //float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput;
            float availableTorque = carTorque;
            Vector3 accelerationForce = accelDir * availableTorque;

            carRigidBody.AddForceAtPosition(accelerationForce, tireTransform.position);

            // Show Acceleration Force
            if (showAccelerationForce) {
                Debug.DrawLine(tireTransform.position, tireTransform.position + accelerationForce, Color.blue);
            }
        }
    }
}
