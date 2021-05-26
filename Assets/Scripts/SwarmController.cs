using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SwarmController : MonoBehaviour
{
    // Start is called before the first frame update
    public float Mass {
        get => _mass;
        set {
            _mass = value;
            UpdateLinksParameter(link => link.mass = _mass);
        }
    }
    public float Damping {
        get => _damping;
        set {
            _damping = value;
            UpdateLinksParameter(link => link.damping = _damping);
        }
    }
    public float Stiffness {
        get => _stiffness;
        set {
            _stiffness = value;
            UpdateLinksParameter(link => link.stiffness = _stiffness);
        }
    }
    public float Gain {
        get => _gain;
        set {
            _gain = value;
            UpdateLinksParameter(link => link.kGain = _gain);
        }
    }

    public Vector3 defaultCorrectionLimit;
    public Transform nearDestination; // some target to follow (but not too far from centroid)
    public GameObject[] drones = new GameObject[4];
    [SerializeField]
    public Vector2[] desiredDistances = new Vector2[4];
    public Transform SwarmCentroid;
    private List<ImpedanceController[]> links =  new List<ImpedanceController[]>();
    [SerializeField]
    private float _mass = 1f, _damping = 2f, _stiffness = 1f, _gain = 1f;
    void Start()
    {
        // head
        links.Add(new ImpedanceController[] {
            SetupLink(drones[0].AddComponent<ImpedanceController>(), nearDestination.transform, logs: true)});
        // right
        links.Add(new ImpedanceController[] {
            SetupLink(drones[1].AddComponent<ImpedanceController>(), drones[0].transform)});
        // left
        links.Add(new ImpedanceController[] {
            SetupLink(drones[2].AddComponent<ImpedanceController>(), drones[0].transform),
            SetupLink(drones[2].AddComponent<ImpedanceController>(), drones[1].transform)});
        // tail
        links.Add(new ImpedanceController[] {
            SetupLink(drones[3].AddComponent<ImpedanceController>(), drones[1].transform),
            SetupLink(drones[3].AddComponent<ImpedanceController>(), drones[2].transform)
        });
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3[] desired = {
            SwarmCentroid.InverseTransformPoint(nearDestination.transform.position),
            drones[0].transform.localPosition,
            drones[0].transform.localPosition,
            0.5f*(drones[1].transform.localPosition + drones[2].transform.localPosition)
        };
        for (int i = 0; i < links.Count; ++i) {
            UpdateGoal(drones[i], links[i], desired[i], desiredDistances[i]);
        }
        RotateSwarm(20f * Time.deltaTime);
    }

    void UpdateGoal(
        GameObject drone,
        ImpedanceController[] links,
        Vector3 desiredPosition,
        Vector2 desiredDistance)
    {
        Vector3 correctionPos = Vector3.zero, correctionVel = Vector3.zero;
        foreach(var link in links) {
            (var onePosCorrection, var oneVelCorrection) = link.GetCorrection();
            correctionPos += onePosCorrection;
            correctionVel += oneVelCorrection;
        }
        correctionPos.x = -Mathf.Abs(correctionPos.x);
        desiredPosition.x -= desiredDistance.x;
        desiredPosition.z += desiredDistance.y;
        // drone.transform.Translate()
        drone.transform.localPosition = desiredPosition + correctionPos;
        // var rb = drone.GetComponent<Rigidbody>();
        // rb.AddForce(correctionVel * Mass, ForceMode.VelocityChange);
        // rb.velocity.Normalize();
    }

    void RotateSwarm(float eulerYaw) {
        SwarmCentroid?.Rotate(0f, eulerYaw, 0f);
    }
    ImpedanceController SetupLink(ImpedanceController link, Transform obstacle, bool logs=false) 
    {
        link.printLogs = logs;
        link.obstacle = obstacle;
        link.mass = Mass;
        link.damping = Damping;
        link.stiffness = Stiffness;
        link.kGain = Gain;
        link.correctionLimit = defaultCorrectionLimit;
        return link;
    }

    void UpdateLinksParameter(System.Action<ImpedanceController> callback)
    {
        foreach(var drone_links in links) {
            foreach(var link in drone_links) {
                callback(link);
            }
        }
    }
}
