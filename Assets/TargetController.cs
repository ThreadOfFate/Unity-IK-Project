using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetController : MonoBehaviour
{
    // Update is called once per frame
    void Update()
    {
        transform.position +=
            (Input.GetAxis("Horizontal") * Vector3.right
             +
             Input.GetAxis("Vertical") * Vector3.up)
            * Time.deltaTime;
    }
}
