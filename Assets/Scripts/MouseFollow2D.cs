using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MouseFollow2D : MonoBehaviour
{
    public float offGroundHeight = 0.5f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out var hit)) {
            if (hit.transform.tag != "Drone") {
                transform.position = hit.point + Vector3.up * offGroundHeight;
            }
        }
    }
}
