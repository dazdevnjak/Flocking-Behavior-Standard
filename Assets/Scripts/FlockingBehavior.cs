using UnityEngine;
using System.Collections.Generic;

public class FlockingBehavior : MonoBehaviour
{
    public string obstacleTag = "Obstacle";

    private List<GameObject> _boids = new List<GameObject>();
    private List<GameObject> _obstacles = new List<GameObject>();

    [Header("Boids")]
    public GameObject boidPrefab;
    public float spawnRadius = 20.0f;
    public int boidCount = 100;

    [Header("General settings")]
    [Range(0.1f, 20f)] public float minSpeed = 5.0f;
    [Range(0.1f, 20f)] public float maxSpeed = 10.0f;
    [Range(0.1f, 2f)] public float deceleration = 0.1f;

    [Range(1.0f, 20f)] public float attackingThreshold = 10.0f;
    [Range(1.0f, 20f)] public float attackingDuration = 20.0f;
    [Range(1.0f, 10f)] public float maxSteerForce = 5.0f;

    [Header("Distances")]
    [Range(0.1f, 10f)] public float arrivalDistance = 2.0f;
    [Range(0.1f, 20f)] public float avoidanceDistance = 3.0f;
    [Range(0.1f, 20f)] public float pursuitDistance = 14.0f;
    [Range(0.1f, 20f)] public float evasionDistance = 6.0f;
    [Range(0.1f, 20f)] public float obstacleDistance = 5.0f;
    [Range(0.1f, 20f)] public float neighborDistance = 10.0f;
    [Range(0.1f, 20f)] public float wanderDistance = 4.0f;

    [Header("Weights")]
    [Range(0f, 10f)] public float noiseWeight = 0.2f;
    [Range(0f, 10f)] public float wanderWeight = 0.2f;
    [Range(0f, 10f)] public float evasionWeight = 4.0f;
    [Range(0f, 10f)] public float pursuitWeight = 3.0f;
    [Range(0f, 10f)] public float cohesionWeight = 0.8f;
    [Range(0f, 10f)] public float alignmentWeight = 1.0f;
    [Range(0f, 50)] public float avoidanceWeight = 20.0f;
    [Range(0f, 10f)] public float separationWeight = 5.0f;
    [Range(0f, 100f)] public float boundsWeight = 100.0f;

    [Header("World bounds")]
    public Vector3 boundsCenter = Vector3.zero;
    public Vector3 boundsSize = new Vector3(50, 50, 50);

    private Bounds _worldBounds;
    private void Awake()
    {
        int numberOfEvaders = (int)(boidCount * 0.9f);
        _worldBounds = new Bounds(boundsCenter, boundsSize);

        for (int i = 0; i < boidCount; i++)
        {
            bool isPursuer = (i >= numberOfEvaders);
            boidPrefab.transform.localScale = isPursuer ? new Vector3(6.0f, 6.0f, 6.0f) : new Vector3(3.0f, 3.0f, 3.0f);

            Vector3 position = transform.position + Random.insideUnitSphere * spawnRadius;
            GameObject boidGO = Instantiate(boidPrefab, position, Quaternion.identity);

            Boid boid = boidGO.GetComponent<Boid>();
            if (boid != null) boid.Initialize(minSpeed, maxSpeed, isPursuer);

            _boids.Add(boidGO);
        }
        LoadAllObstacles();
    }
    private void Update()
    {
        foreach (var boidObj in _boids)
        {
            var boid = boidObj.GetComponent<Boid>();
            if (boid == null) continue;

            Vector3 steeringForce = Vector3.zero;

            steeringForce += Separation(boid) * separationWeight;
            steeringForce += Alignment(boid) * alignmentWeight;
            steeringForce += Cohesion(boid) * cohesionWeight;

            steeringForce += Intent(boid);
            steeringForce += NoiseForce(boid) * noiseWeight;
            steeringForce += CalculateObstacleAvoidance(boid) * avoidanceWeight;
            steeringForce += CalculateBoundsSteer(boid, _worldBounds) * boundsWeight;

            Vector3 desiredVelocity = Vector3.ClampMagnitude(boid.velocity + steeringForce * Time.deltaTime, maxSpeed);
            
            boid.velocity = Vector3.Lerp(boid.velocity, desiredVelocity, 0.2f);
            boid.transform.position += boid.velocity * Time.deltaTime;

            if (boid.velocity.sqrMagnitude > 0.001f)
            {
                Vector3 desiredForward = boid.velocity.normalized;
                boid.transform.forward = Vector3.Slerp(
                    boid.transform.forward,
                    desiredForward,
                    boid.rotationSpeed * Time.deltaTime
                );
            }
        }
    }
    /**
    * Determines the primary steering intent for the given boid, based on its role (pursuer or evader).
    * 
    * If the boid is a pursuer and not on cooldown, it will pursue the nearest valid target for a fixed duration.
    * If currently in a chase, it will continue to pursue the same target for the specified attacking duration.
    * If the boid is an evader and detects a nearby pursuer, it will execute an evade behavior.
    * Otherwise, the boid will wander.
    * 
    * The function manages pursuit cooldowns and ensures the pursuer targets remain consistent during each chase.
    *
    * @param boid The boid whose intent is being calculated.
    * @return Steering force vector representing the boid's intent (pursuit, evade, or wander).
    */
    private Vector3 Intent(Boid boid)
    {
        Vector3 force = Vector3.zero;

        Boid closestBoidToPursue = null, closestBoidToEvade = null;
        float closestPursuitDistance = float.MaxValue, closestEvasionDistance = float.MaxValue;

        foreach (var otherObj in _boids)
        {
            var other = otherObj.GetComponent<Boid>();
            if (other == null || other == boid) continue;

            float distance = Vector3.Distance(boid.transform.position, other.transform.position);

            //Evade
            if (!boid.isPursuer && other.isPursuer)
            {
                if (distance <= evasionDistance && distance < closestEvasionDistance)
                {
                    closestBoidToEvade = other;
                    closestEvasionDistance = distance;
                }
            }

            //Pursuit
            else if (boid.isPursuer && !other.isPursuer)
            {
                bool offCooldown = (Time.time - boid.lastAttackTime > attackingThreshold);
                if (offCooldown && distance <= pursuitDistance && distance < closestPursuitDistance)
                {
                    closestBoidToPursue = other;
                    closestPursuitDistance = distance;
                }
            }
        }

        bool inChase = boid.isPursuer && (Time.time - boid.chaseDuration < attackingDuration);

        if (inChase && boid.pursuitTarget != null)
            force = Pursuit(boid, boid.pursuitTarget) * pursuitWeight;
        else if (boid.isPursuer && closestBoidToPursue != null)
        {
            boid.pursuitTarget = closestBoidToPursue;
            boid.chaseDuration = Time.time;
            boid.lastAttackTime = Time.time;
            force = Pursuit(boid, boid.pursuitTarget) * pursuitWeight;
        }
        else if (!boid.isPursuer && closestBoidToEvade != null)
            force = Evade(boid, closestBoidToEvade) * evasionWeight;
        else
            force = Wander(boid) * wanderWeight;

        if (boid.isPursuer && Time.time - boid.chaseDuration >= attackingDuration)
            boid.pursuitTarget = null;

        return force;
    }
    /**
    * Calculates the cohesion steering force for a boid to move toward the center of mass of nearby boids.
    *
    * @param boid The boid for which the cohesion force is calculated.
    * @return Steering force vector that moves the boid toward the average position of its neighbors within NeighborDistance.
    */
    private Vector3 Cohesion(Boid boid)
    {
        Vector3 force = boid.transform.position;
        int count = 1;
        foreach (var otherObj in _boids)
        {
            var other = otherObj.GetComponent<Boid>();
            if (other == null || other == boid || boid.isPursuer != other.isPursuer) continue;

            float distance = Vector3.Distance(boid.transform.position, other.transform.position);
            if (distance <= neighborDistance)
            {
                force += other.transform.position;
                count++;
            }
        }

        if (count > 1)
        {
            force = ((force / count) - boid.transform.position).normalized * boid.speed - boid.velocity;
            force = Vector3.ClampMagnitude(force, maxSteerForce);
        }

        return force;
    }
    /**
    * Calculates the alignment steering force for a boid to match the direction of neighboring boids.
    *
    * @param boid The boid for which the alignment force is calculated.
    * @return Steering force vector that aligns the boid's velocity with its neighbors within NeighborDistance.
    */
    private Vector3 Alignment(Boid boid)
    {
        Vector3 force = Vector3.zero;
        int count = 0;
        foreach (var otherObj in _boids)
        {
            var other = otherObj.GetComponent<Boid>();
            if (other == null || other == boid || boid.isPursuer != other.isPursuer) continue;

            float distance = Vector3.Distance(boid.transform.position, other.transform.position);
            if (distance <= neighborDistance)
            {
                force += other.velocity;
                count++;
            }
        }

        if (count > 0)
        {
            force = (force / count).normalized * boid.speed - boid.velocity;
            force = Vector3.ClampMagnitude(force, maxSteerForce);
        }

        return force;
    }
    /**
    * Calculates the separation steering force for a boid to avoid crowding.
    *
    * @param boid The boid for which the separation force is calculated.
    * @return Steering force vector that repels the boid from nearby neighbors within AvoidanceDistance.
    */
    private Vector3 Separation(Boid boid)
    {
        Vector3 force = Vector3.zero;
        int count = 0;
        foreach (var otherObj in _boids)
        {
            var other = otherObj.GetComponent<Boid>();
            if (other == null || other == boid || boid.isPursuer != other.isPursuer) continue;

            Vector3 direction = boid.transform.position - other.transform.position;
            float distance = Vector3.Distance(boid.transform.position, other.transform.position);

            if (distance <= avoidanceDistance && distance > 0.01f)
            {
                float scaler = Mathf.Clamp(1.0f - distance / avoidanceDistance, 0.0f, 1.0f);
                force += direction.normalized * scaler;
                count++;
            }
        }

        if (count > 0)
        {
            force = (force / count).normalized * boid.speed - boid.velocity;
            force = Vector3.ClampMagnitude(force, maxSteerForce);
        }

        return force;
    }
    /**
    * Generates a noise vector based on Perlin noise and the boid's unique offset.
    *
    * @param boid The boid for which to generate the noise.
    * @return Vector3 pseudo-random noise force.
    */
    private Vector3 NoiseForce(Boid boid)
    {
        float noiseX = Mathf.PerlinNoise(Time.time / 100.0f, boid.noiseOffset) * 2.0f - 1.0f;
        float noiseY = Mathf.PerlinNoise(Time.time / 120.0f, boid.noiseOffset + 50.0f) * 2.0f - 1.0f;
        float noiseZ = Mathf.PerlinNoise(Time.time / 90.0f, boid.noiseOffset - 50.0f) * 2.0f - 1.0f;
        return new Vector3(noiseX, noiseY, noiseZ);
    }
    /**
    * Finds all GameObjects in the scene tagged "Obstacle" and adds them
    * to the internal list for later avoidance checks.
    */
    private void LoadAllObstacles()
    {
        GameObject[] taggedObjects = GameObject.FindGameObjectsWithTag("Obstacle");
        _obstacles.AddRange(taggedObjects);
    }
    /**
    * Computes a steering force that keeps a boid inside the given axis-aligned bounds.
    * If the boid comes within {@code margin} units of any face, it steers back toward
    * the center at full speed.
    *
    * @param boid the Boid whose position and speed are used
    * @param bounds the axis-aligned Bounds the boid should remain within
    * @param margin the distance from each bounds face at which steering begins (default 3.0f)
    * @return a steering Vector3 (magnitude = boid.Speed) directing the boid back inside,
    * or Vector3.zero if no correction is needed
    */
    private Vector3 CalculateBoundsSteer(Boid boid, Bounds bounds, float margin = 3.0f)
    {
        Vector3 force = Vector3.zero;

        force.x = boid.transform.position.x < bounds.min.x + margin ? 1.0f : boid.transform.position.x > bounds.max.x - margin ? -1.0f : 0.0f;
        force.y = boid.transform.position.y < bounds.min.y + margin ? 1.0f : boid.transform.position.y > bounds.max.y - margin ? -1.0f : 0.0f;
        force.z = boid.transform.position.z < bounds.min.z + margin ? 1.0f : boid.transform.position.z > bounds.max.z - margin ? -1.0f : 0.0f;

        if (force != Vector3.zero)
            return force.normalized * boid.speed;
        return Vector3.zero;
    }
    /**
     * Samples a number of random ray directions within the boid’s field of view
     * to detect nearby obstacles, and computes an avoidance steering force. If less
     * than half the samples hit an obstacle, the boid steers toward the direction
     * with the greatest free distance. Otherwise it blends a repulsion vector with
     * its current forward direction.
     *
     * @param boid the Boid to calculate avoidance for
     * @param fov field of view in degrees over which to sample directions (default 90.0f)
     * @param numOfFleers number of rays to cast within the FOV (default 30)
     * @return a Vector3 steering force (clamped to MaxSteerForce) that avoids obstacles
     * while maintaining speed
     */
    private Vector3 CalculateObstacleAvoidance(Boid boid, float fov = 90.0f, int numOfFleers = 30)
    {
        Vector3 force = Vector3.zero;
        Vector3 avoidance = Vector3.zero;

        float maxFreeDist = 0f;
        Vector3 bestDirection = boid.transform.forward;
        int count = 0;

        for (int i = 0; i < numOfFleers; i++)
        {
            float freeDist;

            float angleY = (Random.value - 0.5f) * fov;
            float angleX = (Random.value - 0.5f) * fov;

            Quaternion rot = Quaternion.AngleAxis(angleY, boid.transform.up) * Quaternion.AngleAxis(angleX, boid.transform.right);
            Vector3 dir = rot * boid.transform.forward;

            if (Physics.Raycast(boid.transform.position, dir, out RaycastHit hit, obstacleDistance))
            {
                if (_obstacles.Contains(hit.collider.gameObject))
                {
                    avoidance += (boid.transform.position - hit.point).normalized * (obstacleDistance - hit.distance) / obstacleDistance;
                    count++;
                }
                freeDist = hit.distance;
            }
            else
            {
                freeDist = obstacleDistance;
            }

            if (freeDist > maxFreeDist)
            {
                maxFreeDist = freeDist;
                bestDirection = dir;
            }
        }

        if (count < numOfFleers * 0.5f)
            force = bestDirection.normalized * boid.speed - boid.velocity;
        else
        {
            Vector3 blended = Vector3.Slerp(boid.transform.forward, avoidance.normalized, 0.8f);
            force = blended * boid.speed - boid.velocity;
        }

        return Vector3.ClampMagnitude(force, maxSteerForce);
    }
    /**
     * Calculates the steering force to seek a target.
     *
     * @param boid The boid.
     * @param speed The desired speed.
     * @return Steering force toward the target.
     */
    private Vector3 Seek(Boid boid)
    {
        Vector3 direction = boid.target - boid.transform.position;
        return direction.magnitude <= arrivalDistance
            ? Arrive(boid)
            : Vector3.ClampMagnitude((direction.normalized * boid.speed - boid.velocity), maxSteerForce);
    }
    /**
     * Calculates the steering force to flee from a target.
     *
     * @param boid The boid.
     * @param speed The desired speed.
     * @return Steering force away from the target.
     */
    private Vector3 Flee(Boid boid)
    {
        Vector3 direction = boid.transform.position - boid.target;
        return direction.magnitude <= arrivalDistance
        ? Arrive(boid)
            : Vector3.ClampMagnitude((direction.normalized * boid.speed - boid.velocity), maxSteerForce);
    }
    /**
     * Calculates arrival steering with deceleration near the target.
     *
     * @param boid The boid seeking to arrive.
     * @return Steering force.
     */
    private Vector3 Arrive(Boid boid)
    {
        Vector3 direction = boid.target - boid.transform.position;
        float distance = direction.magnitude;
        float speed = distance / deceleration;
        float clippedSpeed = Mathf.Min(speed, maxSpeed);
        return Vector3.ClampMagnitude((direction.normalized * clippedSpeed - boid.velocity), maxSteerForce);
    }
    /**
    * Generates a new target using wander behavior.
    *
    * @param boid The boid to update.
    * @return Steering force toward new wander target.
    */
    private Vector3 Wander(Boid boid)
    {
        if (Vector3.Distance(boid.transform.position, boid.target) < 1.0f)
        {
            float angle = Hash(boid.noiseOffset + Time.time * 0.5f) * Mathf.PI * 2f;
            float height = Hash(boid.noiseOffset * 3.1f + Time.time) * 2.0f - 1.0f;

            Vector3 randomDir = new Vector3(
                    Mathf.Cos(angle) * Mathf.Sqrt(1.0f - height * height),
                    height,
                    Mathf.Sin(angle) * Mathf.Sqrt(1.0f - height * height)
                );

            boid.target = boid.transform.position + randomDir * wanderDistance;
        }

        return Seek(boid);
    }
    /**
     * Predicts and seeks the future position of a target.
     *
     * @param pursuer The boid pursuing.
     * @param target The target boid.
     * @return Steering force toward the future position.
     */
    private Vector3 Pursuit(Boid pursuer, Boid target)
    {
        float lookAhead = Vector3.Distance(pursuer.transform.position, target.transform.position) / (pursuer.speed + target.speed);
        Vector3 futurePosition = target.transform.position + target.velocity * lookAhead;
        pursuer.target = futurePosition;
        return Seek(pursuer);
    }
    /**
    * Predicts and flees from the future position of a threat.
    *
    * @param evader The boid fleeing.
    * @param threat The threat boid.
    * @return Steering force away from the future position.
    */
    private Vector3 Evade(Boid evader, Boid threat)
    {
        float lookAhead = Vector3.Distance(evader.transform.position, threat.transform.position) / (evader.speed + threat.speed);
        Vector3 futurePosition = threat.transform.position + threat.velocity * lookAhead;
        evader.target = futurePosition;
        return Flee(evader);
    }

    /**
    * Hashes a float input into a pseudo-random float in [0,1].
    *
    * @param n Input float.
    * @return Hashed pseudo-random float.
    */
    public float Hash(float n) => Frac(Mathf.Sin(n) * 43758.5453f);
    /**
    * Returns the fractional (decimal) part of a float value.
    * 
    * For example, Frac(3.14) returns 0.14, and Frac(-2.7) returns 0.3.
    *
    * @param v The input float value.
    * @return The fractional part of v, always in the range [0, 1).
    */
    private float Frac(float v) => v - Mathf.Floor(v);
}