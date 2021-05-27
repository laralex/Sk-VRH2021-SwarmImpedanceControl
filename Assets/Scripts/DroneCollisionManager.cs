using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneCollisionManager : MonoBehaviour
{
    public Rigidbody rigidbody;
    public bool hasCollided;

    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
        hasCollided = false;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Obstacle")
        {
            hasCollided = true;
        }
    }
}
