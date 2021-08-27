using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Rendering;

public class Boid_Spawner : MonoBehaviour
{
    public int boidsPerInterval;
    public int boidsToSpawn;
    public float interval;
    public float cohesionBias;
    public float separationBias;
    public float alignmentBias;
    public float targetBias;
    public float perceptionRadius;
    public float3 target;
    public float maxSpeed;
    public float step;
    public int cellSize;
    public GameObject prefab;

    private EntityManager entitymanager;
    private Entity entity;
    private float elapsedTime;
    private int totalSpawnedBoids;
    private EntityArchetype ea;
    private float3 currentPosition;
    private BlobAssetStore bas;
    private GameObjectConversionSettings gocs;
    private Entity convertedEntity;

    private void Start()
    {
        totalSpawnedBoids = 0;
        entitymanager = World.DefaultGameObjectInjectionWorld.EntityManager;
        currentPosition = this.transform.position;
        bas = new BlobAssetStore();
        gocs = GameObjectConversionSettings.FromWorld(World.DefaultGameObjectInjectionWorld, bas);
        convertedEntity = GameObjectConversionUtility.ConvertGameObjectHierarchy(prefab, gocs);
        Destroy(prefab);
    }

    private void Update()
    {
        elapsedTime += Time.deltaTime;
        if (elapsedTime >= interval)
        {
            elapsedTime = 0;
            for (int i = 0; i <= boidsPerInterval; i++)
            {
                if (totalSpawnedBoids == boidsToSpawn)
                {
                    break;
                }
                Entity e;
                if (i==0)
                {
                    e = convertedEntity;
                }
                else
                {
                    e = entitymanager.Instantiate(convertedEntity);
                }

                entitymanager.SetComponentData(e, new Translation
                {
                    Value = currentPosition
                });
                entitymanager.AddComponentData(e, new Boid_ComponentData
                {
                    velocity = math.normalize(UnityEngine.Random.insideUnitSphere) * maxSpeed,
                    perceptionRadius = perceptionRadius,
                    speed = maxSpeed,
                    step = step,
                    cohesionBias = cohesionBias,
                    separationBias = separationBias,
                    alignmentBias = alignmentBias,
                    target = target,
                    targetBias = targetBias,
                    cellSize = cellSize
                });
                totalSpawnedBoids++;
            }
        }
    }

    private void OnDestroy()
    {
        bas.Dispose();
    }
}