using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random=UnityEngine.Random;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


public class MoveToGoalAgentRandom : Agent
{
    public Transform target;
    public Transform[] obstacles = new Transform[4];
    public float forceMultiplier = 1f;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floor;
    Rigidbody rBody;
    Vector3 goalPosition;
    Vector3 agentPosition;
    private GameObject[] drones;
    private Vector2[] desiredDistances;
    public float maxStretch = 75f;
    private float initDistance;
    private float targetDistance;
    private float prevTargetDistance;
    private bool hasCollided;
    private int counter = 0;
    private float time = 0;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();

        // Retrieve swarm variables
        drones = this.GetComponent<SwarmController>().drones;
        desiredDistances = this.GetComponent<SwarmController>().desiredDistances;
        initDistance = desiredDistances[1][0];
    }

    private float GetStretch()
    {
        float ratio = desiredDistances[1][0] / initDistance;
        return Mathf.Log(ratio, 2f) * 100f;
    }
    
    private void SetStretch(float stretch)
    {
        desiredDistances[0][0] = -initDistance * Mathf.Pow(2f, stretch / 100f);
        desiredDistances[1][0] = initDistance * Mathf.Pow(2f, stretch / 100f);
        desiredDistances[1][1] = initDistance * Mathf.Pow(2f, -stretch / 100f);
        desiredDistances[2][0] = initDistance * Mathf.Pow(2f, stretch / 100f);
        desiredDistances[2][1] = -initDistance * Mathf.Pow(2f, -stretch / 100f);
        desiredDistances[3][0] = initDistance * Mathf.Pow(2f, stretch / 100f);
    }

    private void AddStretch(float stretch)
    {
        desiredDistances[0][0] *= Mathf.Pow(2f, stretch / 100f); //*= alpha;
        desiredDistances[1][0] *= Mathf.Pow(2f, stretch / 100f); //*= alpha;
        desiredDistances[1][1] *= Mathf.Pow(2f, -stretch / 100f); ///= alpha;
        desiredDistances[2][0] *= Mathf.Pow(2f, stretch / 100f); //*= alpha;
        desiredDistances[2][1] *= Mathf.Pow(2f, -stretch / 100f); ///= alpha;
        desiredDistances[3][0] *= Mathf.Pow(2f, stretch / 100f); //*= alpha;
    }

    public override void OnEpisodeBegin()
    {
        time = 0;

        // Zero the swarm velocities
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;

        // Zero the drones velocities
        foreach (GameObject drone in drones)
        {
            drone.transform.rotation = Quaternion.identity;
            Rigidbody rb = drone.GetComponent<DroneCollisionManager>().rigidbody;
            rb.angularVelocity = Vector3.zero;
            rb.velocity = Vector3.zero;
        }

        // Remove any stretch
        SetStretch(0);

        //this.transform.localPosition = new Vector3(0.33f, 0.5f, 2.54f);

        // Randomize obstacle positions
        // TODO: Debug that
        // GenerateObstacle();

        // Randomize agent and target positions
        Populate();

        // Compute the initial distance to the target
        targetDistance = Vector3.Distance(
            this.transform.position,
            target.position
        );
        // targetDistance = Math.Abs(this.transform.position.z - target.position.z);
    }

    private void FinishFailure()
    {
        SetReward(-1.0f);
        floor.material = loseMaterial;
        EndEpisode();
        counter++;
        // Debug.Log(counter);
    }

    private void FinishSuccess()
    {
        SetReward(10.0f);
        floor.material = winMaterial;
        EndEpisode();
    }

    private void CheckForCollision()
    {
        bool hasCollided = false;
        foreach (GameObject drone in drones)
        {
            if (drone.GetComponent<DroneCollisionManager>().hasCollided)
            {
                hasCollided = true;
                FinishFailure();
                break;
            }
        }
        // Reset drone collision markers
        if (hasCollided)
        {
            foreach (GameObject drone in drones)
            {
                drone.GetComponent<DroneCollisionManager>().hasCollided = false;
            }
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Vector3 controlSignal = Vector3.zero;
        float stretch;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.y = 0f; // actionBuffers.ContinuousActions[1];
        controlSignal.z = actionBuffers.ContinuousActions[1]; //[2];
        stretch = actionBuffers.ContinuousActions[2]; //[3];

        // Stretch the swarm
        AddStretch(stretch);

        // Move the swarm
        rBody.AddForce(
            controlSignal.x * forceMultiplier,
            controlSignal.y * forceMultiplier, // rBody.mass * Physics.gravity.magnitude,
            controlSignal.z * forceMultiplier,
            ForceMode.Force
        );

        // Check for drone collisions
        CheckForCollision();

        // TODO: Change this for colliders
        time += Time.deltaTime;
        if (this.transform.localPosition.x < -2.85f
            || this.transform.localPosition.x > 2.85f
            || this.transform.localPosition.z < -2.85f
            || this.transform.localPosition.z > 2.85f
            || this.transform.localPosition.y < 0.25f
            || this.transform.localPosition.y > 0.95f
            || time > 60f)
        {
            FinishFailure();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Goal")
        {
            FinishSuccess();
        }
        // Obstacle collision detection is delegated to drones
        // else if (other.gameObject.tag == "Obstacle")
        // {
        //     FinishFailure();
        // }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Stabilize the swarm controller
        this.rBody.transform.rotation = Quaternion.identity;

        // Register the target and agent positions
        sensor.AddObservation(target.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        // Register the swarm geometry
        float stretch = GetStretch();
        sensor.AddObservation(stretch);

        // Penalize excessive stretching
        if (Math.Abs(stretch) > maxStretch)
        {
            AddReward(-(Math.Abs(stretch) - maxStretch) / 100f);
        }

        // foreach (GameObject drone in drones)
        // {
        //     sensor.AddObservation(drone.transform.localPosition.x);
        //     sensor.AddObservation(drone.transform.localPosition.z);
        // }

        // Register the agent velocity
        sensor.AddObservation(rBody.velocity);

        // Give a reward according to the change in distance to the target
        prevTargetDistance = targetDistance;
        targetDistance = Vector3.Distance(
            this.transform.position,
            target.position
        );
        // targetDistance = Math.Abs(this.transform.position.z - target.position.z);
        AddReward(prevTargetDistance - targetDistance);

        // Existential penalty
        AddReward(-0.001f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        // Control x/z-axes movements
        continuousActionsOut[0] = Input.GetAxisRaw("Horizontal");
        continuousActionsOut[2] = Input.GetAxisRaw("Vertical");

        // Control y-axis movements
        if (Input.GetKey(KeyCode.LeftShift))
        {
            continuousActionsOut[1] = 1;
        }
        else if (Input.GetKey(KeyCode.LeftControl))
        {
            continuousActionsOut[1] = -1;
        }
        else
        {
            continuousActionsOut[1] = 0f;
        }

        // Control swarm deformations
        if (Input.GetKey(KeyCode.C))
        {
            continuousActionsOut[3] = 1;
        }
        else if (Input.GetKey(KeyCode.V))
        {
            continuousActionsOut[3] = -1;
        }
        else
        {
            continuousActionsOut[3] = 0f;
        }
    }

    private void GenerateObstacle()
    {
        bool obstaclesSpawned = false;
        while (!obstaclesSpawned)
        {
            Vector3 obs1 = new Vector3(Random.Range(-2.25f, 2.25f), 0.5f, Random.Range(-2.25f, 2.25f));
            Vector3 obs2 = new Vector3(Random.Range(-2.25f, 2.25f), 0.5f, Random.Range(-2.25f, 2.25f));
            Vector3 obs3 = new Vector3(Random.Range(-2.25f, 2.25f), 0.5f, Random.Range(-2.25f, 2.25f));
            Vector3 obs4 = new Vector3(Random.Range(-2.25f, 2.25f), 0.5f, Random.Range(-2.25f, 2.25f));

            if ((obs1 - obs2).magnitude < 1.5
                || (obs1 - obs3).magnitude < 1.5
                || (obs1 - obs4).magnitude < 1.5
                || (obs2 - obs3).magnitude < 1.5
                || (obs2 - obs4).magnitude < 1.5
                || (obs3 - obs4).magnitude < 1.5)
            {
                continue;
            }
            else
            {
                obstacles[0].localPosition = obs1;
                obstacles[1].localPosition = obs2;
                obstacles[2].localPosition = obs3;
                obstacles[3].localPosition = obs4;
                obstaclesSpawned = true;
            }
        }
    }
    
    // TODO: Debug
    // private void Populate()
    // {
    //     bool goalspawned = false;
    //     while (!goalspawned)
    //     {
    //         goalPosition = new Vector3(
    //             Random.Range(-2.75f, 2.75f),
    //             Random.Range(0.3f, 0.7f),
    //             Random.Range(-2.75f, -1f)
    //         );
    //         agentPosition = new Vector3(
    //             Random.Range(-2f, 2f),
    //             Random.Range(0.3f, 0.7f),
    //             Random.Range(1f, 2f)
    //         );
    //         foreach (Transform obstacle in obstacles)
    //         {
    //             if ((goalPosition - obstacle.localPosition).magnitude < 2
    //                 || (agentPosition - obstacle.localPosition).magnitude < 2)
    //             {
    //                 continue;
    //             }
    //             else
    //             {
    //                 target.localPosition = goalPosition;
    //                 this.transform.localPosition = agentPosition;
    //                 goalspawned = true;
    //             }
    //         }
    //     }
    // }
    // Simplified version
    private void Populate()
    {
        goalPosition = new Vector3(
            Random.Range(-2.5f, 2.5f),
            0.5f,
            // Random.Range(0.3f, 0.7f),
            Random.Range(-2.25f, -1.75f)
        );
        agentPosition = new Vector3(
            Random.Range(-2f, 2f),
            0.5f,
            // Random.Range(0.3f, 0.7f),
            Random.Range(1f, 2f)
        );

        target.localPosition = goalPosition;
        this.transform.localPosition = agentPosition;
    }
}


