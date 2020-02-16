using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FABRIKController : MonoBehaviour
{
    /// <summary>
    /// Target for Fabrik to follow
    /// </summary>
    public Transform target;
    /// <summary>
    /// Number of bones affected by this script
    /// </summary>
    public int boneChainLength;
    /// <summary>
    /// Limits the number of iterations FABRIK is allow to do
    /// </summary>
    public int iterationLimit;
    /// <summary>
    /// Object which handle FABRIK Implimentation
    /// </summary>
    public FABRIK fabrik;
    

    // Start is called before the first frame update
    void Start()
    {
        fabrik = new FABRIK(this.transform, iterationLimit, boneChainLength);
    }

    // Update is called once per frame
    void LateUpdate()
    {
        fabrik.SetTarget(target);
        fabrik.Resolve();
    }
}
