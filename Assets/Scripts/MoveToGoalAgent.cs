using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


public class MoveToGoalAgent : Agent
{
    public float speed = 100;
    public Transform Target;
    public Transform[] obs;
    public float forceMultiplier = 0.01f;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floor;
    Vector3 goalPosition;
    private int counter = 0;
    private float time = 0;
    //private float prevDistance;
    Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        time = 0;
        // If the Agent fell, zero its momentum
        this.rBody.angularVelocity = Vector3.zero;
        this.rBody.velocity = Vector3.zero;
        this.transform.localPosition = new Vector3(0.33f, 0.5f, 2.54f);
        //float prevDistance = Vector3.Distance(this.transform.position, Target.transform.position);

        //Target.localPosition = new Vector3(-1.5f, 0.54f, -1.54f);
        // Move the target to a new spot
        //So that target and obstacle not collide
   
        bool goalspawned = false;
        while (!goalspawned)
        {
            goalPosition = new Vector3(
                Random.Range(-2.75f, 2.75f),
                Random.Range(0.5f, 0.6f),
                Random.Range(-2.75f, 2.75f)
            );

            if ((goalPosition - obs[0].localPosition).magnitude < 1
                || (goalPosition - obs[1].localPosition).magnitude < 1
                || (goalPosition - obs[2].localPosition).magnitude < 1
                || (goalPosition - obs[3].localPosition).magnitude < 1)
            {
                continue;
            }
            else
            {
                Target.localPosition = goalPosition;
                goalspawned = true;
            }
        }
        //If to try moving obstacle
        //obs[0].localPosition = new Vector3(Random.Range(-2.5f, 2.5f), 0.5f, -0.5f);
        //Debug.Log(prevDistance);
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

        //rBody.mass* Physics.gravity.magnitude = hovering case
        //rBody.mass* Physics.gravity.magnitude + controlSignal.y * forceMultiplier = Thrust case

        rBody.AddForce(
            controlSignal.x * forceMultiplier,
            rBody.mass * Physics.gravity.magnitude + controlSignal.y * 0.25f * forceMultiplier,
            controlSignal.z * forceMultiplier, ForceMode.Force
        );

        // Moving to target
        //float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);
        time += Time.deltaTime;

        //Limits of playfield and height
        if (this.transform.localPosition.x < -2.85f
            || this.transform.localPosition.x > 2.85f
            || this.transform.localPosition.z < -2.85f
            || this.transform.localPosition.z > 2.85f
            || this.transform.localPosition.y < 0.25f
            || this.transform.localPosition.y > 0.75f
            || time > 60f)
        {
            FinishFailure();
        }
    }

    //rewards
    //We want to add more policy here to fasten the training process
    //The main reward and failure
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Goal")
        {
            FinishSuccess();
        }
        else if (other.gameObject.tag == "Obstacle")
        {
            FinishFailure();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(Target.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        // Agent velocity
        sensor.AddObservation(rBody.velocity);
        // sensor.AddObservation(rBody.velocity.x);
        // sensor.AddObservation(rBody.velocity.y);
        // sensor.AddObservation(rBody.velocity.z);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxisRaw("Horizontal");
        continuousActionsOut[2] = Input.GetAxisRaw("Vertical");
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
    }
}

