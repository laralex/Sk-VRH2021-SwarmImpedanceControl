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
    private bool isHeuristic = false;
    private List<GameObject> obstaclesList;
    private int fieldSideLength = 6; // TODO: Deduct it with the field scale
    private int counter = 0;
    private float time = 0;
    private float forceMultiplier = 1f;
    private Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
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
        time = 0;
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
        // Actions, size = 2
        Debug.Log("In Action");
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.y = actionBuffers.ContinuousActions[1];
        controlSignal.z = actionBuffers.ContinuousActions[2];

        // For the y-axis:
        // | Hovering case: rBody.mass * Physics.gravity.magnitude
        // | Thrust case: controlSignal.y * forceMultiplier + rBody.mass * Physics.gravity.magnitude
        // TODO: Add a check to deactivate gravity if in mode `Heuristics`
        Debug.Log(rBody.mass * Physics.gravity.magnitude * (isHeuristic ? 1f : 0f));
        rBody.AddForce(
            controlSignal.x * forceMultiplier,
            controlSignal.y * forceMultiplier
            + rBody.mass * Physics.gravity.magnitude * (isHeuristic ? 1f : 0f),
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
        Debug.Log("In Observation");
        // Target and Agent positions
        sensor.AddObservation(target.transform.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        // Agent velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.y);
        sensor.AddObservation(rBody.velocity.z);

        // Give a reward according to the distance to the target
        // prevDistance = distance;
        // distance = Vector3.Distance(
        //     this.transform.position,
        //     target.transform.position
        // );
        // AddReward((distance - prevDistance) / 100f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.Log("In Heuristic");
        isHeuristic = true;
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
