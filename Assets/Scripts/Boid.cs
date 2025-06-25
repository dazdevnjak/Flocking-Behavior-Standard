using Unity.VisualScripting.Antlr3.Runtime;
using UnityEngine;
using UnityEngine.UIElements;

public class Boid : MonoBehaviour
{
    [HideInInspector] public Vector3 target;
    [HideInInspector] public Boid pursuitTarget;
    [HideInInspector] public Vector3 velocity;

    [HideInInspector] public float speed;
    [HideInInspector] public float rotationSpeed;
    [HideInInspector] public float noiseOffset;
    [HideInInspector] public float lastAttackTime;
    [HideInInspector] public float chaseDuration;

    [HideInInspector] public bool isPursuer;

    public void Initialize(float minSpeed, float maxSpeed, bool isPursuer)
    {
        this.isPursuer = isPursuer;
        target = transform.position;
        speed = Random.Range(minSpeed, maxSpeed);
        velocity = Vector3.zero;
        rotationSpeed = Random.Range(minSpeed, maxSpeed);
        noiseOffset = Random.value * 1000.0f;
        lastAttackTime = -999.0f;
        chaseDuration = -1.0f;
        pursuitTarget = null;
    }
}