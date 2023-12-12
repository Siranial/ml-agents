using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class BlockTrigger : MonoBehaviour
{
    [Header("Collider Tag To Detect")]
    public string tagToDetect = "agent"; //collider tag to detect

    [Header("Target Placement")]
    public float spawnRadius; //The radius in which a target can be randomly spawned.
    
    private Vector3 m_startingPos; //the starting position of the target
    
    // Start is called before the first frame update
    void OnEnable()
    {
        m_startingPos = transform.position;
        MoveBlockToRandomPosition();
    }

    /// <summary>
    /// Moves block to a random position within specified radius.
    /// </summary>
    public void MoveBlockToRandomPosition()
    {
        var newBlockPos = m_startingPos + (Random.insideUnitSphere * spawnRadius);
        newBlockPos.y = 1f;
        transform.position = newBlockPos;
    }

    // Start is called before the first frame update
    void OnTriggerEnter (Collider col)
    {
        if (col.transform.parent.CompareTag(tagToDetect))
        {
            col.transform.parent.tag = "block";
            MoveBlockToRandomPosition();
        }
    }
}
