using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tire : MonoBehaviour
{
    [Header("Suspension")]
    [SerializeField] private float suspensionRestDist;
    [SerializeField] private float springStrength;
    [SerializeField] private float springDamper;

    [Header("Debug Options")]
    [SerializeField] private bool showGroundCheckRaycast;
    [SerializeField] private bool showSuspensionForce;

    private bool rayDidHit;
    private RaycastHit tireRay;

    private Rigidbody carRigidBody;

    private void Awake() {
        carRigidBody = GetComponentInParent<Rigidbody>();
    }

    private void Update() {
        // Check if grounded
        GroundRaycast();

        // Show Ground Raycast
        if (showGroundCheckRaycast) {
            Debug.DrawRay(transform.position, Vector3.down * suspensionRestDist, Color.yellow);
        }

        // suspension spring force
        if (rayDidHit) {
            ApplySpringForce();
        }
    }

    private void GroundRaycast() {
        rayDidHit = Physics.Raycast(transform.position, Vector3.down, out tireRay, suspensionRestDist);
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
            Debug.DrawRay(transform.position, suspensionForce, Color.green);
        }
    }
}
