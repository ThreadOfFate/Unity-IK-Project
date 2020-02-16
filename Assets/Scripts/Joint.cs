using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Joint : MonoBehaviour
{
    /// <summary>
    /// The rotational constriants, these values are used to create a circle,ellipsoidal or parabolic shaped restriction for the rotation
    /// </summary>
    [Range(0f,89.99f)]
    public float xp, xn, yp, yn;
    /// <summary>
    /// Determines how much a joint can twist along the axis of the directional vector to the next bone
    /// </summary>
    [Range(0f, 360f)]
    public float orientationalRestricts;

}
