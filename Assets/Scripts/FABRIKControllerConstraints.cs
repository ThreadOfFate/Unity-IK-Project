using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FABRIKControllerConstraints : MonoBehaviour
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
    /// Object which handle FABRIK Implimentation with constraints
    /// </summary>
    public FABRIKUsingJoints fabrikJoints;
    
    /// <summary>
    /// The default joint used if a bone does not have a joint set up
    /// </summary>
    [SerializeField]
    public Joint joints;

    // Start is called before the first frame update
    void Start()
    {
        fabrikJoints = new FABRIKUsingJoints(this.transform, iterationLimit, boneChainLength, joints);
    }

    // Update is called once per frame
    void LateUpdate()
    {
        fabrikJoints.SetTarget(target);
        fabrikJoints.Resolve();
    }
}
