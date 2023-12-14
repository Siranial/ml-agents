using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class CrawlPusherAgent : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    [Tooltip(
        "The speed the agent will try to match.\n\n" +
        "TRAINING:\n" +
        "For VariableSpeed envs, this value will randomize at the start of each training episode.\n" +
        "Otherwise the agent will try to match the speed set here.\n\n" +
        "INFERENCE:\n" +
        "During inference, VariableSpeed agents will modify their behavior based on this value " +
        "whereas the CrawlerDynamic & CrawlerStatic agents will run at the speed specified during training "
    )]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15; //The max walking speed

    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")][Space(10)] public Transform body;
    public Transform leg0Upper;
    public Transform leg0Lower;
    public Transform leg1Upper;
    public Transform leg1Lower;
    public Transform leg2Upper;
    public Transform leg2Lower;
    public Transform leg3Upper;
    public Transform leg3Lower;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    OrientationCubeController m_OrientationCubeBlock;
    DirectionIndicator m_DirectionIndicatorBlock;

    [Header("Foot Grounded Visualization")]
    [Space(10)]
    public bool useFootGroundedVisualization;

    public MeshRenderer foot0;
    public MeshRenderer foot1;
    public MeshRenderer foot2;
    public MeshRenderer foot3;
    public Material groundedMaterial;
    public Material unGroundedMaterial;


    //The ground object
    [Header("The Ground Object")]
    public GameObject ground;

    //The block to be pushed to the goal.
    [Header("Block to Push Towards Goal")]
    public Transform blockPrefab;
    public Transform m_Block; //Block the agent will push
    //private Vector3 blockStartPos;
    //private Vector3 prevBlockPos;
    //private Vector3 agentStartPos;
    //private float prevDotAlignment;

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position); //spawn target
        SpawnBlock(blockPrefab, transform.position); //spawn block
        //blockStartPos = m_Block.transform.position;
        //prevBlockPos = blockStartPos;
        //prevDotAlignment = Vector3.Dot(cubeForward, blockForward);

        m_OrientationCube = GetComponentsInChildren<OrientationCubeController>()[0];
        m_DirectionIndicator = GetComponentsInChildren<DirectionIndicator>()[0];
        m_JdController = GetComponent<JointDriveController>();

        m_OrientationCubeBlock = GetComponentsInChildren<OrientationCubeController>()[1];
        m_DirectionIndicatorBlock = GetComponentsInChildren<DirectionIndicator>()[1];

        //agentStartPos = m_OrientationCubeBlock.transform.position;

        //Setup each body part
        m_JdController.SetupBodyPart(body);
        m_JdController.SetupBodyPart(leg0Upper);
        m_JdController.SetupBodyPart(leg0Lower);
        m_JdController.SetupBodyPart(leg1Upper);
        m_JdController.SetupBodyPart(leg1Lower);
        m_JdController.SetupBodyPart(leg2Upper);
        m_JdController.SetupBodyPart(leg2Lower);
        m_JdController.SetupBodyPart(leg3Upper);
        m_JdController.SetupBodyPart(leg3Lower);
    }

    /// <summary>
    /// Use the ground's bounds to pick a random spawn position.
    /// </summary>
    public Vector3 GetRandomSpawnPos()
    {
        var foundNewSpawnLocation = false;
        var randomSpawnPos = Vector3.zero;
        while (foundNewSpawnLocation == false)
        {
            var randomPosX = Random.Range(-30.0f, 30.0f);
            var randomPosZ = Random.Range(-30.0f, 30.0f);
            randomSpawnPos = ground.transform.position + new Vector3(randomPosX, 1.5f, randomPosZ);
            if (Physics.CheckBox(randomSpawnPos, new Vector3(2.5f, 0.01f, 2.5f)) == false)
            {
                foundNewSpawnLocation = true;
            }
        }
        return randomSpawnPos;
    }

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Spawns a block prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnBlock(Transform prefab, Vector3 pos)
    {
        m_Block = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        //Set our goal walking speed
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);

        //blockStartPos = m_Block.transform.position;
        //prevBlockPos = blockStartPos;
        //agentStartPos = m_OrientationCubeBlock.transform.position;
        transform.tag = "agent";
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;
        var blockForward = m_OrientationCubeBlock.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        //rotation delta
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        //Add pos of block relative to orientation cube
        sensor.AddObservation(m_OrientationCubeBlock.transform.InverseTransformPoint(m_Block.transform.position));

        //Add status of having picked up block
        sensor.AddObservation(transform.CompareTag("block"));

        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
            sensor.AddObservation(1);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        // Pick a new target joint rotation
        bpDict[leg0Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg1Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg2Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg3Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg0Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg1Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg2Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg3Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);

        // Update joint strength
        bpDict[leg0Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg0Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Lower].SetJointStrength(continuousActions[++i]);

        // Penalty given each step to encourage agent to finish task quickly.
        //AddReward(-1f / MaxStep);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        // If enabled the feet will light up green when the foot is grounded.
        // This is just a visualization and isn't necessary for function
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[leg0Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[leg1Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[leg2Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[leg3Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        //Direction to goal
        var cubeForward = m_OrientationCube.transform.forward;
<<<<<<< Updated upstream
        //Direction to block
        var blockForward = m_OrientationCubeBlock.transform.forward;
=======
        var blockForward = m_OrientationCube_Block.transform.forward;

        //if (attached)
        //{
        //    AttachBlock();
        //}

        if(!attached && Vector3.Distance(m_OrientationCube.transform.position, block_transform.position) < 5.0f)
        {
            attached = true;
            AttachBlock();
        }
        if(attached && block_transform.localPosition.y <= this.transform.localPosition.y + body.transform.localPosition.y)
        {
            EndEpisode();
        }

>>>>>>> Stashed changes
        
        //Reward agent for getting closer to block
        //var agentDistToBlockReward = Mathf.Clamp((1f - (Vector3.Distance(m_Block.transform.position, m_OrientationCubeBlock.transform.position) / Vector3.Distance(blockStartPos, agentStartPos))), 0f, 1f);

        //Calculate how aligned the to block and to goal vectors are
        //This signifies if the agent is in a good position to push the block to the goal
        //var dotAlignment = Vector3.Dot(cubeForward, blockForward);

        //Reward agent for being in a position suitable for pushing block to goal
        //var dotAlignmentReward = Mathf.Clamp(dotAlignment, .1f, 0.75f) / 0.75f;

        //var blockDistToTarget = Vector3.Distance(m_Block.transform.position, m_Target.transform.position);
        //The block's velocity towards the goal for this cycle
        //var blockVelocityRelativeToTarget = (prevBlockDistToTarget - blockDistToTarget) / Time.deltaTime;

        //var blockDistanceMoved = Vector3.Distance(prevBlockPos, m_Block.transform.position) / Time.deltaTime;
        //var blockMoveReward = Mathf.Clamp(100f*blockDistanceMoved, 0f, .5f);
        //prevBlockPos = m_Block.transform.position;
        //var blockMoveReward = Mathf.Clamp(Vector3.Distance(m_Block.transform.position, blockStartPos), 0f, .5f) - .25f;

        //Reward agent for moving block towards goal
        //var blockVelocitytToTargetReward = Mathf.Clamp(Mathf.Pow(blockVelocityRelativeToTarget, 2), 0f, 0.5f);

        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = 0f;

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = 0f;

        var holdingBlockReward = 0f;

        //Heading towards block
        if (transform.CompareTag("agent")){
            matchSpeedReward = GetMatchingVelocityReward(blockForward * TargetWalkingSpeed, GetAvgVelocity());
            lookAtTargetReward = (Vector3.Dot(blockForward, body.forward) + 1) * .5F;
            holdingBlockReward = 0f;
        } 
        //Heading towards goal
        else {
            matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, GetAvgVelocity());
            lookAtTargetReward = (Vector3.Dot(cubeForward, body.forward) + 1) * .5F;
            holdingBlockReward = .2f;
        }
        AddReward(matchSpeedReward * lookAtTargetReward + holdingBlockReward);
        if(transform.parent.name == "CrawlPushPlatform") Debug.Log("tag: " + transform.tag + " Speed R: " + matchSpeedReward + " Look R: " + lookAtTargetReward + " Holding R: " + holdingBlockReward);
    }

    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        //Orientation towards goal
        m_OrientationCube.UpdateOrientation(body, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
<<<<<<< Updated upstream
        //Orientation towards block
        m_OrientationCubeBlock.UpdateOrientation(body, m_Block);
        if (m_DirectionIndicatorBlock)
        {
            m_DirectionIndicatorBlock.MatchOrientation(m_OrientationCubeBlock.transform);
        }
=======
    }

    void AttachBlock()
    {
        block_transform.localPosition = this.transform.localPosition + body.transform.localPosition + new Vector3(0.0f, 1.5f, 0.0f) ;
>>>>>>> Stashed changes
    }

    /// <summary>
    ///Returns the average velocity of all of the body parts
    ///Using the velocity of the body only has shown to result in more erratic movement from the limbs
    ///Using the average helps prevent this erratic movement
    /// </summary>
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        Vector3 avgVel = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        avgVel = velSum / numOfRb;
        return avgVel;
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

<<<<<<< Updated upstream
=======
    public float GetApproachingBlockReward()
    {
        // Define a maximum distance for calculation
        float maxDistance = 80.0f; // This should be set according to your environment

        // Calculate the current distance between the agent and the block
        float currentDistance = Vector3.Distance(m_OrientationCube.transform.position, block_transform.position);

        // Normalize the distance
        float normalizedDistance = Mathf.Clamp(currentDistance, 0, maxDistance) / maxDistance;

        // Calculate the reward using a declining sigmoid-shaped curve
        // This reward will approach 1 as the agent gets closer to the block and approach 0 as it moves away
        float reward = Mathf.Pow(1 - Mathf.Pow(normalizedDistance, 2), 2);

        return reward;
    }

    public float GetBlockToTargetReward()
    {
        // Define a maximum distance for calculation
        float maxDistance = 80.0f; // This should be set according to your environment

        // Calculate the current distance between the agent and the block
        float currentDistance = Vector3.Distance(block_transform.position, m_Target.transform.position);

        // Normalize the distance
        float normalizedDistance = Mathf.Clamp(currentDistance, 0, maxDistance) / maxDistance;

        // Calculate the reward using a declining sigmoid-shaped curve
        // This reward will approach 1 as the agent gets closer to the block and approach 0 as it moves away
        float reward = Mathf.Pow(1 - Mathf.Pow(normalizedDistance, 2), 2);

        return reward;
    }

    public void Missin_Complete()
    {
        //AddReward(5f * MaxStep);
        EndEpisode();
    }

>>>>>>> Stashed changes
    /// <summary>
    /// Agent got block to target
    /// </summary>
    public void TouchedTarget()
    {
        if(transform.CompareTag("block")) AddReward(10f);

        EndEpisode();
    }
}
