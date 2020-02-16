using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// A FABRIK implimentation base off 
/// FABRIK: A fast, iterative solver for the Inverse Kinematics problem - Andreas Aristidou & Joan Lasenby
/// </summary>
[SerializeField]
public class FABRIK
{
    
    //Used vector3 and Quaterion over a target transform to allow for better use with Ray Casting etc
    /// <summary>
    /// The target postion in world space
    /// </summary>
    private Vector3 targetPosition;

    /// <summary>
    /// The target rotation
    /// </summary>
    private Quaternion targetRotation;

    /// <summary>
    /// All the bones affected by this script
    /// </summary>
    private Transform[] bones;

    /// <summary>
    /// The starting positions of the bones
    /// </summary>
    private Quaternion[] startRotations;

    /// <summary>
    /// The length of each bone
    /// </summary>
    private float[] bonesLength;

    /// <summary>
    /// The total length of all bones
    /// </summary>
    private float completeLength;

    /// <summary>
    /// The starting positions of all bones
    /// </summary>
    private Vector3[] positions;

    /// <summary>
    /// The starting direction of each bone
    /// </summary>
    private Vector3[] startDirection;

    /// <summary>
    /// The starting rotation of each bone
    /// </summary>
    private Quaternion[] startRotation;

    /// <summary>
    /// The starting rotation of the target
    /// </summary>
    private Quaternion startRotationTarget;

    /// <summary>
    /// The root bone, this bone should not move
    /// </summary>
    private Transform root;

    /// <summary>
    /// The value used for floating point error
    /// </summary>
    private float EPSILON = 0.001f;

    /// <summary>
    /// Limits the number of iterations for the script
    /// </summary>
    private float iterationLimit = 5;

    /// <summary>
    /// The number of bones this script should affect
    /// This will start from the transform and will recur up through the parents
    /// </summary>
    private int boneChainLength = 0;

    // Start is called before the first frame update




    /// <summary>
    /// Constructor for FABRIK using Joints
    /// </summary>
    /// <param name="startBone"> The end bone</param>
    /// <param name="newIterationLimit"> The number of Iterations before it stops</param>
    /// <param name="newboneChainLength"> How many bones this will affect </param>
    public FABRIK(Transform startBone,float newIterationLimit, int newboneChainLength)
    {
        //Set up values
        iterationLimit = newIterationLimit;
        boneChainLength = newboneChainLength;
        bones = new Transform[boneChainLength];
        positions = new Vector3[bones.Length];
        bonesLength = new float[bones.Length - 1];
        startDirection = new Vector3[bones.Length];
        startRotation = new Quaternion[bones.Length];

        //Finds the root
        root = startBone;
        for (int i = 0; i <= bones.Length - 1; i++)
        {
            if (root == null)
            {
                throw new UnityException("Chain Length is bigger then parent layers");
            }
            root = root.parent;
        }

        //Makes sure there is a target
        if (targetPosition == null || targetRotation == null)
        {
            throw new UnityException("There is no target");
        }

        //Sets up positions relative to the root bone to avoid possible issues from moving the root bone while resolving
        Transform current = startBone;
        completeLength = 0;
        for (int i = bones.Length - 1; i >= 0; i--)
        {
            bones[i] = current;
            startRotation[i] = Quaternion.Inverse(current.rotation) * root.rotation;

            if (i == bones.Length - 1)
            {
                startDirection[i] = (Quaternion.Inverse(root.rotation) * (targetPosition - root.position)) - (Quaternion.Inverse(root.rotation) * (current.position - root.position));
            }
            else
            {
                startDirection[i] = (Quaternion.Inverse(root.rotation) * (bones[i + 1].position - root.position)) - (Quaternion.Inverse(root.rotation) * (current.position - root.position));
                bonesLength[i] = startDirection[i].magnitude;
                completeLength += bonesLength[i];
            }

            current = current.parent;
        }



    }

    /// <summary>
    /// Uses Alogrithem 1 from FABRIK: A fast, iterative solver for the Inverse Kinematics problem - Andreas Aristidou & Joan Lasenby
    /// When called will procedurally have the bones follow the target
    /// </summary>
    public void Resolve()
    {
        

        #region Check If solvable

        for (int i = 0; i < bones.Length; i++)
        {
            positions[i] = Quaternion.Inverse(root.rotation) * (bones[i].position - root.position);
        }



        //Checks if the target can be reached comparing the distacne between the root bone and the target to the total length of all the bones
        if ((targetPosition - Quaternion.Inverse(root.rotation) * (bones[0].position - root.position)).sqrMagnitude >= completeLength * completeLength)
        {
            //just strech it
            Vector3 direction = (targetPosition - positions[0]).normalized;
            //set everything after root
            for (int i = 1; i < positions.Length; i++)
            {
                positions[i] = positions[i - 1] + direction * bonesLength[i - 1];
            }

        }
        #endregion
        else
        {


            for (int iteration = 0; iteration < iterationLimit; iteration++)
            {
                #region Backwards

                for (int i = positions.Length - 1; i > 0; i--)
                {
                    if (i == positions.Length - 1)
                    {
                        positions[i] = targetPosition;
                    }
                    else
                    {
                        positions[i] = positions[i + 1] + (positions[i] - positions[i + 1]).normalized * bonesLength[i];
                    }
                }

                #endregion

                #region Forwards
                for (int i = 1; i < positions.Length; i++)
                {
                    positions[i] = positions[i - 1] + (positions[i] - positions[i - 1]).normalized * bonesLength[i - 1];
                }
                

                #endregion

                if ((positions[positions.Length - 1] - targetPosition).sqrMagnitude < EPSILON * EPSILON)
                {
                    break;
                }
            }

        }

        //set position & rotation
        for (int i = 0; i < positions.Length; i++)
        {
            if (i == positions.Length - 1)
            {
                bones[i].rotation = root.rotation * (Quaternion.Inverse(targetRotation) * Quaternion.Inverse(startRotation[i]));
            }
            else
            {
                bones[i].rotation = root.rotation * (Quaternion.FromToRotation(startDirection[i], positions[i + 1] - positions[i]) * Quaternion.Inverse(startRotation[i]));
            }
            
            bones[i].position = root.rotation * positions[i] + root.position;
        }


    }

    /// <summary>
    /// A method that sets the target for the IK
    /// </summary>
    public void SetTarget(Transform target)
    {
        targetPosition = Quaternion.Inverse(root.rotation) * (target.position - root.position);
        targetRotation = Quaternion.Inverse(target.rotation) * root.rotation;
    }


}
