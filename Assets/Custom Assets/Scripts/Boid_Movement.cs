using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;

public class Boid_Movement : SystemBase
{
    public NativeMultiHashMap<int, Boid_ComponentData> cellVsEntityPositions;

    public static int GetUniqueKeyForPosition(float3 position, int cellSize)
    {
        return (int)((15 * math.floor(position.x / cellSize)) + (17 * math.floor(position.y / cellSize)) + (19 * math.floor(position.z / cellSize)));
    }

    protected override void OnCreate()
    {
        cellVsEntityPositions = new NativeMultiHashMap<int, Boid_ComponentData>(0, Allocator.Persistent);
    }

    protected override void OnUpdate()
    {
        ComponentTypeHandle<Translation> translationTypeHandle = GetComponentTypeHandle<Translation>();
        ComponentTypeHandle<Boid_ComponentData> bcdTypeHandle = GetComponentTypeHandle<Boid_ComponentData>();
        ComponentTypeHandle<Rotation> rotationTypeHandle = GetComponentTypeHandle<Rotation>();
        EntityQuery bb_query = GetEntityQuery(typeof(Translation), ComponentType.ReadOnly<Boid_ComponentData>());
        EntityQuery bc_query = GetEntityQuery(typeof(Translation), typeof(Rotation), ComponentType.ReadWrite<Boid_ComponentData>());
        float deltaTime = Time.DeltaTime;

        cellVsEntityPositions.Clear();
        if (bb_query.CalculateEntityCount() > cellVsEntityPositions.Capacity)
        {
            cellVsEntityPositions.Capacity = bb_query.CalculateEntityCount();
        }

        BucketBoids bbjob = new BucketBoids
        {
            cellVsEntityPositionsParallel = cellVsEntityPositions.AsParallelWriter(),
            translationTypeHandle = translationTypeHandle,
            bcdTypeHandle = bcdTypeHandle
        };
        this.Dependency = bbjob.ScheduleParallel(bb_query, 10, this.Dependency);

        Boid_Calculate bcjob = new Boid_Calculate
        {
            translationTypeHandle = translationTypeHandle,
            rotationTypeHandle = rotationTypeHandle,
            bcdTypeHandle = bcdTypeHandle,
            deltaTime = deltaTime,
            cellVsEntityPositionsForJob = cellVsEntityPositions
        };
        this.Dependency = bcjob.ScheduleParallel(bc_query, 10, this.Dependency);
    }

    [BurstCompatible]
    struct BucketBoids : IJobEntityBatch
    {
        public NativeMultiHashMap<int, Boid_ComponentData>.ParallelWriter cellVsEntityPositionsParallel;
        public ComponentTypeHandle<Translation> translationTypeHandle;
        [ReadOnly] public ComponentTypeHandle<Boid_ComponentData> bcdTypeHandle;

        public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
        {
            NativeArray<Translation> chunkTranslation = batchInChunk.GetNativeArray(translationTypeHandle);
            NativeArray<Boid_ComponentData> chunkBcd = batchInChunk.GetNativeArray(bcdTypeHandle);

            for (int i=0; i<batchInChunk.Count; i++)
            {
                Translation trans = chunkTranslation[i];
                Boid_ComponentData bcd = chunkBcd[i];
                Boid_ComponentData bcValues = new Boid_ComponentData();
                bcValues = bcd;
                bcValues.currentPosition = trans.Value;
                cellVsEntityPositionsParallel.Add(GetUniqueKeyForPosition(trans.Value, bcd.cellSize), bcValues);
            }
        }
    }

    [BurstCompatible]
    struct Boid_Calculate : IJobEntityBatch
    {
        public ComponentTypeHandle<Translation> translationTypeHandle;
        public ComponentTypeHandle<Boid_ComponentData> bcdTypeHandle;
        public ComponentTypeHandle<Rotation> rotationTypeHandle;
        [ReadOnly] public NativeMultiHashMap<int, Boid_ComponentData> cellVsEntityPositionsForJob;
        public float deltaTime;

        public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
        {
            NativeArray<Translation> chunkTranslation = batchInChunk.GetNativeArray(translationTypeHandle);
            NativeArray<Boid_ComponentData> chunkBcd = batchInChunk.GetNativeArray(bcdTypeHandle);
            NativeArray<Rotation> chunkRotation = batchInChunk.GetNativeArray(rotationTypeHandle);

            for (int i = 0; i < batchInChunk.Count; i++)
            {
                Translation trans = chunkTranslation[i];
                Boid_ComponentData bc = chunkBcd[i];
                Rotation rot = chunkRotation[i];
                int key = GetUniqueKeyForPosition(trans.Value, bc.cellSize);
                NativeMultiHashMapIterator<int> nmhKeyIterator;
                Boid_ComponentData neighbour;
                int total = 0;
                float3 separation = float3.zero;
                float3 alignment = float3.zero;
                float3 coheshion = float3.zero;

                if (cellVsEntityPositionsForJob.TryGetFirstValue(key, out neighbour, out nmhKeyIterator))
                {
                    do
                    {
                        if (!trans.Value.Equals(neighbour.currentPosition) && math.distance(trans.Value, neighbour.currentPosition) < bc.perceptionRadius)
                        {
                            float3 distanceFromTo = trans.Value - neighbour.currentPosition;
                            separation += (distanceFromTo / math.distance(trans.Value, neighbour.currentPosition));
                            coheshion += neighbour.currentPosition;
                            alignment += neighbour.velocity;
                            total++;
                        }
                    } while (cellVsEntityPositionsForJob.TryGetNextValue(out neighbour, ref nmhKeyIterator));
                    if (total > 0)
                    {
                        coheshion = coheshion / total;
                        coheshion = coheshion - (trans.Value + bc.velocity);
                        coheshion = math.normalize(coheshion) * bc.cohesionBias;

                        separation = separation / total;
                        separation = separation - bc.velocity;
                        separation = math.normalize(separation) * bc.separationBias;

                        alignment = alignment / total;
                        alignment = alignment - bc.velocity;
                        alignment = math.normalize(alignment) * bc.alignmentBias;
                    }

                    Boid_ComponentData bcnew = new Boid_ComponentData();
                    bcnew = bc;
                    bcnew.acceleration += (coheshion + alignment + separation);
                    if (!float.IsNaN(bcnew.acceleration.x))
                    {
                        bcnew.velocity = bc.velocity + bcnew.acceleration;
                        bcnew.velocity = math.normalize(bcnew.velocity) * bc.speed;
                    }
                    bcnew.acceleration = math.normalize(bc.target - trans.Value) * bc.targetBias;
                    chunkBcd[i] = bcnew;

                    chunkRotation[i] = new Rotation
                    {
                        Value = math.slerp(rot.Value, quaternion.LookRotation(math.normalize(bc.velocity), math.up()), deltaTime * 10)
                    };
                    chunkTranslation[i] = new Translation
                    {
                        Value = math.lerp(trans.Value, (trans.Value + bc.velocity), deltaTime * bc.step)
                    };
                }
            }
        }
    }

    protected override void OnDestroy()
    {
        cellVsEntityPositions.Dispose();
    }
}
