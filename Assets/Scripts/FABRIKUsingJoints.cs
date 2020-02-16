using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A FABRIK implimentation base off 
/// FABRIK: A fast, iterative solver for the Inverse Kinematics problem - Andreas Aristidou & Joan Lasenby
/// </summary>
[SerializeField]
public class FABRIKUsingJoints
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
    /// The root bone's position
    /// </summary>
    private Vector3 rootPosition;

    /// <summary>
    /// The root bone's rotation
    /// </summary>
    private Quaternion rootRotation;

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

    /// <summary>
    /// The default Joint used if a bone doesn't have a joint
    /// </summary>
    private Joint defaultJoint;




    /// <summary>
    /// Constructor for FABRIK using Joints
    /// </summary>
    /// <param name="startBone"> The end bone</param>
    /// <param name="newIterationLimit"> The number of Iterations before it stops</param>
    /// <param name="newboneChainLength"> How many bones this will affect </param>
    /// <param name="newJoint"> The fallback joint if a bone doesn't have a joint </param>
    public FABRIKUsingJoints(Transform startBone, float newIterationLimit, int newboneChainLength, Joint newJoint)
    {
        //Set up values
        defaultJoint = newJoint;
        
        iterationLimit = newIterationLimit;
        boneChainLength = newboneChainLength;
        bones = new Transform[boneChainLength];
        positions = new Vector3[bones.Length];
        bonesLength = new float[bones.Length - 1];
        startDirection = new Vector3[bones.Length];
        startRotation = new Quaternion[bones.Length];

        //Finds the root
        root = startBone;
        for (int i = 1; i <= bones.Length - 1; i++)
        {
            if (root == null)
            {
                throw new UnityException("Chain Length is bigger then parent layers");
            }
            root = root.parent;
        }
        rootPosition = root.position;
        rootRotation = root.rotation;

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
            startRotation[i] = Quaternion.Inverse(current.rotation) * rootRotation;
            if (i == bones.Length - 1)
            {
                startDirection[i] = (Quaternion.Inverse(rootRotation) * (targetPosition - rootPosition)) - (Quaternion.Inverse(rootRotation) * (current.position - rootPosition));
            }
            else
            {
                startDirection[i] = (Quaternion.Inverse(rootRotation) * (bones[i + 1].position - rootPosition)) - (Quaternion.Inverse(rootRotation) * (current.position - rootPosition));
                bonesLength[i] = startDirection[i].magnitude;
                completeLength += bonesLength[i];
            }
            current = current.parent;
        }
    }

    /// <summary>
    /// Uses Alogrithem 1, 2 and 3 from FABRIK: A fast, iterative solver for the Inverse Kinematics problem - Andreas Aristidou & Joan Lasenby
    /// When called will procedurally have the bones follow the target
    /// </summary>
    public void Resolve()
    {
        Joint joints;

        #region Check If solvable

        for (int i = 0; i < bones.Length; i++)
        {
            positions[i] = Quaternion.Inverse(rootRotation) * (bones[i].position - rootPosition);
        }



        //Checks if the target can be reached comparing the distacne between the root bone and the target to the total length of all the bones
        if ((targetPosition - Quaternion.Inverse(rootRotation) * (bones[0].position - rootPosition)).sqrMagnitude >= completeLength * completeLength)
        {
            Vector3 direction = (targetPosition - positions[0]).normalized;
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

                //Restricts the Rotation
                #region RotationalConstraints
                positions[positions.Length - 1] = targetPosition;
                for (int i = positions.Length - 2; i > 0; i--)
                {

                    Vector3 lineL = (positions[i + 1] - positions[i]).normalized;
                    Vector3 O = (Vector3.Dot(targetPosition, lineL) / Vector3.Dot(lineL, lineL)) * lineL;
                    float S = (O - positions[i]).magnitude;

                    Vector3 fakeVector2Target = (Quaternion.FromToRotation(lineL, Vector3.forward) * (targetPosition - O));
                    if (bones[i].GetComponent<Joint>() != null)
                    {
                        joints = bones[i].GetComponent<Joint>();
                    }
                    else
                    {
                        Debug.Log(bones[i].name + " is using the default joint");
                        joints = defaultJoint;
                    }
                    float boundX = 0;
                    float boundY = 0;

                    //This blends the angles for the 4 axis together bepending on the actual targets localtion, this should allow for the creation of a more flexable range of joints over just defining them
                    if (fakeVector2Target.x > 0)
                    {
                        boundX = Mathf.Tan(joints.xp * Mathf.PI / 180) * fakeVector2Target.normalized.x;
                    }
                    else
                    {
                        boundX = Mathf.Tan(joints.xn * Mathf.PI / 180) * fakeVector2Target.normalized.x;
                    }

                    if (fakeVector2Target.y > 0)
                    {
                        boundY = Mathf.Tan(joints.yp * Mathf.PI / 180) * fakeVector2Target.normalized.y;
                    }
                    else
                    {
                        boundY = Mathf.Tan(joints.yn * Mathf.PI / 180) * fakeVector2Target.normalized.y;
                    }


                    //Checks if its in the bounds already
                    if (S * (boundX + boundY) >= fakeVector2Target.magnitude)
                    {
                        positions[i] = positions[i + 1] + (positions[i] - positions[i + 1]).normalized * bonesLength[i];
                    }
                    else
                    {
                        //
                        fakeVector2Target = fakeVector2Target.normalized * (S * (boundX + boundY));
                        positions[i] = positions[i + 1] +
                        ((((Quaternion.Inverse(Quaternion.FromToRotation(lineL, Vector3.forward)) * fakeVector2Target) + O) - positions[i + 1]).normalized
                        * bonesLength[i]);
                    }
                    #endregion
                    

                    #region OrientationalConstraints
                    
                    if (bones[i].GetComponent<Joint>() != null)
                    {
                        joints = bones[i-1].GetComponent<Joint>();
                    }
                    else
                    {
                        Debug.Log(bones[i-1].name + " is using the default joint");
                        joints = defaultJoint;
                    }

                    //float currentAngle = (Quaternion.FromToRotation((positions[i] - positions[i - 1]), Vector3.up) * (rootRotation * (Quaternion.FromToRotation(startDirection[i - 1], positions[i] - positions[i - 1]) * Quaternion.Inverse(startRotation[i - 1])))).eulerAngles.y;
                    ////Debug.Log(currentAngle);
                    //float startAngle = (Quaternion.FromToRotation(startDirection[i - 1], Vector3.up) * startRotation[i - 1]).eulerAngles.y;
                    //Debug.Log(startAngle);
                    float difference =
                        GetTwist((rootRotation * (Quaternion.FromToRotation(startDirection[i - 1], positions[i] - positions[i - 1]) * Quaternion.Inverse(startRotation[i - 1]))), positions[i] - positions[i - 1]).eulerAngles.magnitude -
                        GetTwist(startRotation[i - 1], startDirection[i - 1]).eulerAngles.magnitude;

                    if (Mathf.Abs(difference)
                        > joints.orientationalRestricts)
                    {
                        float theta;
                        if (difference>0)
                        {
                            theta = difference - joints.orientationalRestricts;
                        }
                        else
                        {
                            theta = difference + joints.orientationalRestricts;
                        }
                        
                        Vector3 axis = ((Mathf.Sin((theta) * Mathf.Deg2Rad / 2)) * (positions[i] - positions[i - 1])).normalized;
                        float w = (Mathf.Cos((theta) * Mathf.Deg2Rad / 2));
                        //1^2 = sqrt(x^2 +y^2 + z^2 + w^2)
                        //1 - w^2 = x^2 + y^2 + z^2
                        //This should scale the axis down to be right to fit in the quaternion
                        axis = Mathf.Sqrt(1 - (w * w)) * axis;
                        Vector4 temp = new Vector4(axis.x, axis.y, axis.z, w);

                        Debug.Log(temp.magnitude);


                        positions[i - 1] = positions[i] -
                            (
                            Quaternion.Inverse(new Quaternion(axis.x, axis.y, axis.z, w))
                            * axis
                        );

                    }

                    #endregion

                }

                #endregion

                #region Forwards
                //Position are all relative to the root, so this just resets the 1st position
                positions[0] = Vector3.zero;

                for (int i = 1; i < positions.Length-1; i++)
                {
                    positions[i] = positions[i - 1] + (positions[i] - positions[i - 1]).normalized * bonesLength[i - 1];

                    if (bones[i].GetComponent<Joint>() != null)
                    {
                        joints = bones[i].GetComponent<Joint>();
                    }
                    else
                    {
                        Debug.Log(bones[i].name + " is using the default joint");
                        joints = defaultJoint;
                    }
                    
                    if (bones[i].GetComponent<Joint>() != null)
                    {
                        joints = bones[i - 1].GetComponent<Joint>();
                    }
                    else
                    {
                        Debug.Log(bones[i - 1].name + " is using the default joint");
                        joints = defaultJoint;
                    }

                    float difference =
                        GetTwist((rootRotation * (Quaternion.FromToRotation(startDirection[i - 1], positions[i] - positions[i - 1]) * Quaternion.Inverse(startRotation[i - 1]))), positions[i] - positions[i - 1]).eulerAngles.magnitude -
                        GetTwist(startRotation[i - 1], startDirection[i - 1]).eulerAngles.magnitude;

                    if (Mathf.Abs(difference)
                        > joints.orientationalRestricts)
                    {
                        float theta;
                        if (difference > 0)
                        {
                            theta = difference - joints.orientationalRestricts;
                        }
                        else
                        {
                            theta = difference + joints.orientationalRestricts;
                        }

                        Vector3 axis = ((Mathf.Sin((theta) * Mathf.Deg2Rad / 2)) * (positions[i] - positions[i - 1])).normalized;
                        float w = (Mathf.Cos((theta) * Mathf.Deg2Rad / 2));
                        //1^2 = sqrt(x^2 +y^2 + z^2 + w^2)
                        //1 - w^2 = x^2 + y^2 + z^2
                        //This should scale the axis down to be right to fit in the quaternion
                        axis = Mathf.Sqrt(1 - (w * w)) * axis;
                        Vector4 temp = new Vector4(axis.x, axis.y, axis.z, w);

                        Debug.Log(temp.magnitude);


                        positions[i - 1] = positions[i] -
                            (
                            Quaternion.Inverse(new Quaternion(axis.x, axis.y, axis.z, w))
                            * axis
                        );

                    }


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
                bones[i].rotation = rootRotation * (Quaternion.Inverse(targetRotation) * Quaternion.Inverse(startRotation[i]));
            }
            else
            {
                bones[i].rotation = rootRotation * (Quaternion.FromToRotation(startDirection[i], positions[i + 1] - positions[i]) * Quaternion.Inverse(startRotation[i]));
            }
            
            bones[i].position = rootRotation * positions[i] + rootPosition;
        }


    }


    /// <summary>
    /// A method that sets the target for the IK
    /// </summary>
    public void SetTarget(Transform target)
    {
        
        targetPosition = Quaternion.Inverse(rootRotation) * (target.position - rootPosition);
        targetRotation = Quaternion.Inverse(target.rotation) * rootRotation;
        
    }

    private Quaternion GetTwist(Quaternion rotation, Vector3 newAxis)
    {
        //formula match respectively
        //w + x + y +z
        //Cos(theta/2) + Sin(theta/2)(i + j + k)
        // so baseAxis is actualy ijk with sin(theta/2)
        Vector3 baseAxis = new Vector3(rotation.x, rotation.y, rotation.z);
        //since sin(theta/2) is include in baseAxis this project will result in the relative rotation to eachother (excluding w)
        Vector3 axis = Vector3.Project(baseAxis, newAxis);
        //normalize it to make sure its a valid Quaternion and adjust all the values
        return new Quaternion(axis.x, axis.y, axis.z, rotation.w).normalized;
    }


    

}
