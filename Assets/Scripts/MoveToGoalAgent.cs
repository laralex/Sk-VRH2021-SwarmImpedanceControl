using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random=UnityEngine.Random;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


public class MoveToGoalAgent : Agent
{    
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject obstacle;
    [SerializeField] private int nObstacles;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floor;
    // private bool isHeuristic = false;
    private List<GameObject> obstaclesList;
    private int fieldSideLength = 6;
    private int counter = 0;
    private float forceMultiplier = 1f;
    private float distance;
    private float prevDistance;
    private Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();

        // Retrieve the field scale
        // TODO: Check that
        // GameObject plane = GameObject.Find("Plane");
        // Transform planeTransform = plane.GetComponent<Transform>();
        // fieldSideLength = (int) planeTransform.localScale.x;
        // Debug.Log(fieldSideLength);
    }

    // TODO: Make it nicer
    private List<int> RandomSamples(int n, int max)
    {
        List<int> samples = new List<int>();
        for (int i = 0; i < max; i++) {
            samples.Add(i);
        }
        return samples.OrderBy(x => Random.value).ToList().GetRange(0, n);
    }

    private void AddObstacle(Vector3 position)
    {
        GameObject newObstacle = Instantiate(obstacle) as GameObject;
        newObstacle.transform.parent = obstacle.transform.parent;
        newObstacle.transform.localPosition = position;
        newObstacle.SetActive(true);
        obstaclesList.Add(newObstacle);
    }

    private Vector3 PositionIdToVector(int id, int length) {
        int z = Math.DivRem(id, length, out int x);
        Vector3 position = new Vector3(x, 0, z);
        return position;
    }

    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;

        // Delete previous obstacles, if any
        if (obstaclesList != null) {
            foreach (GameObject obstacle in obstaclesList)
            {
                Destroy(obstacle);
            }
        }

        // Sample some random positions
        List<int> positionIdsList = RandomSamples(
            nObstacles + 2,
            fieldSideLength * fieldSideLength - 1
        );

        // Add obstacles
        obstaclesList = new List<GameObject>();
        for (int i = 0; i < nObstacles; i++) {
            AddObstacle(
                PositionIdToVector(positionIdsList[i], fieldSideLength)
            );
        }

        // Position the agent
        // TODO: Add some randomness within the tile
        this.transform.localPosition = PositionIdToVector(
            positionIdsList[positionIdsList.Count - 2], fieldSideLength
        );

        // Position the target
        // TODO: Add some randomness within the tile
        target.transform.localPosition = PositionIdToVector(
            positionIdsList[positionIdsList.Count - 1], fieldSideLength
        );

        // Compute the initial distance to the target
        distance = Vector3.Distance(
            this.transform.position,
            target.transform.position
        );
    }

    private void FinishFailure()
    {
        SetReward(-1.0f);
        floor.material = loseMaterial;
        EndEpisode();
        counter++;
        Debug.Log(counter);
    }

    private void FinishSuccess()
    {
        SetReward(5.0f);
        floor.material = winMaterial;
        EndEpisode();
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.y = actionBuffers.ContinuousActions[1];
        controlSignal.z = actionBuffers.ContinuousActions[2];

        // For the y-axis:
        // | Hovering case: rBody.mass * Physics.gravity.magnitude
        // | Thrust case: controlSignal.y * forceMultiplier + rBody.mass * Physics.gravity.magnitude
        // TODO: Add a check to deactivate gravity if in mode `Heuristics`
        // Debug.Log(rBody.mass * Physics.gravity.magnitude);
        rBody.AddForce(
            controlSignal.x * forceMultiplier,
            0, //controlSignal.y * forceMultiplier + rBody.mass * Physics.gravity.magnitude,
            controlSignal.z * forceMultiplier,
            ForceMode.Force
        );
    }

    //rewards
    //We want to add more policy here to fasten the training process
    //The main reward and failure
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Target")
        {
            FinishSuccess();
        }
        else if (other.gameObject.tag == "Obstacle"
                || other.gameObject.tag == "Boundary")
        {
            FinishFailure();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(target.transform.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        // Agent velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.y);
        sensor.AddObservation(rBody.velocity.z);

        // Obstacles positions
        // TODO: Automatically set the observation vector size somewhere
        // (for now, observation vector size is hardcoded in the editor)
        foreach (GameObject obstacle in obstaclesList)
        {
            sensor.AddObservation(obstacle.transform.localPosition);
        }
        
        // Give a reward according to the change in distance to the target
        prevDistance = distance;
        distance = Vector3.Distance(
            this.transform.position,
            target.transform.position
        );
        AddReward(prevDistance - distance);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;
        // x/z-axes
        continuousActionsOut[0] = Input.GetAxisRaw("Horizontal");
        continuousActionsOut[2] = Input.GetAxisRaw("Vertical");
        // y-axis
        if (Input.GetKey(KeyCode.LeftShift))
        {
            continuousActionsOut[1] = 1f;
        }
        else if (Input.GetKey(KeyCode.LeftControl))
        {
            continuousActionsOut[1] = -1f;
        }
        else
        {
            continuousActionsOut[1] = 0f;
        }
    }
}
