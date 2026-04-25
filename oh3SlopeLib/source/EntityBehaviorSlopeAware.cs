using System;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Extends entity physics by generating a dynamically interpolated surface normal and distance metric.
    /// Utilizes a virtual sampling sphere to aggregate surrounding block geometry, enabling advanced 
    /// surface-aligned mechanics such as rolling, sliding, or custom IK adaptations.
    /// </summary>
    public class EntityBehaviorSlopeAware : EntityBehavior
    {
        /// <summary>
        /// The normalized, mathematically averaged vector representing the orientation of the surrounding terrain.
        /// </summary>
        public Vec3d SurfaceNormal { get; private set; } = new Vec3d(0, 1, 0);

        /// <summary>
        /// The minimal orthogonal distance from the boundary of the virtual sampling sphere to the resolved surface plane.
        /// </summary>
        public double DistanceToSurface { get; private set; } = 9999.0;

        /// <summary>
        /// The diameter of the virtual boundary sphere used for geometric sampling.
        /// </summary>
        public double CollisionSphereSize { get; set; } = 1.0;

        /// <summary>
        /// The vertical elevation of the sampling sphere's center, relative to the entity's base position.
        /// </summary>
        public double CollisionSphereYOffset { get; set; } = 0.5;

        /// <summary>
        /// The exact Cartesian coordinate residing on the resolved surface plane, derived by projecting the sphere's center.
        /// </summary>
        public Vec3d SurfacePoint { get; private set; } = new Vec3d(0, 0, 0);

        private Vec3d lastSampledPos = new Vec3d(0, -9999, 0);
        private const double SampleThresholdSq = 0.1 * 0.1;

        // Pre-allocated buffers to minimize Garbage Collection overhead during hot-path execution.
        private BlockPos minPos = new BlockPos();
        private BlockPos maxPos = new BlockPos();
        private BlockPos tmpPos = new BlockPos();
        private Vec3d center = new Vec3d();
        private Vec3d averageNormal = new Vec3d();

        private struct FastAABB
        {
            public double x1, y1, z1, x2, y2, z2;
            public int bx, by, bz;
        }
        private FastAABB[] localBoxes = new FastAABB[512];
        private int localBoxCount = 0;

        // Delegate state buffers to avoid closure allocations during spatial queries.
        private double sphereRadius;
        private double minDistanceToEdge;
        private int facesSampled;
        private double closestX;
        private double closestY;
        private double closestZ;
        private double closestNormX;
        private double closestNormY;
        private double closestNormZ;
        private double avgNormX;
        private double avgNormY;
        private double avgNormZ;
        private Action<Block, int, int, int> walkBlocksDelegate;

        // Adjacency cache to minimize chunk data access overhead per block iteration.
        private Block[] neighborBlocks = new Block[6];
        private bool[] neighborBlocksFetched = new bool[6];

        public EntityBehaviorSlopeAware(Entity entity) : base(entity)
        {
            // Cache the delegate once during initialization to prevent implicit heap allocations.
            walkBlocksDelegate = OnBlockWalked;
        }

        public override string PropertyName() => "slopeaware";

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            JsonObject slopelibConfig = null;

            // Attempt to resolve configuration from the entity's global attributes.
            if (properties.Attributes != null && properties.Attributes.KeyExists("slopelib"))
            {
                slopelibConfig = properties.Attributes["slopelib"];
            }
            // Attempt to resolve configuration nested within the behavior's attributes.
            else if (attributes != null && attributes.KeyExists("slopelib"))
            {
                slopelibConfig = attributes["slopelib"];
            }
            // 3. Fall back to the behavior's attributes directly if no "slopelib" wrapper was used
            else if (attributes != null && (attributes.KeyExists("diameter") || attributes.KeyExists("yoffset")))
            {
                slopelibConfig = attributes;
            }

            // Apply resolved configuration or enforce default fallbacks.
            if (slopelibConfig != null)
            {
                CollisionSphereSize = slopelibConfig["diameter"].AsDouble(1.0);
                CollisionSphereYOffset = slopelibConfig["yoffset"].AsDouble(CollisionSphereSize / 2.0);
            }
            else
            {
                CollisionSphereSize = 1.0;
                CollisionSphereYOffset = 0.5;
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            // Ensure the block accessor is available to prevent exceptions during chunk loading edges.
            if (entity?.World?.BlockAccessor == null) return;

            try
            {
                // Calculate delta manually to prevent implicit Vec3d allocations from property getters.
                double dx = entity.Pos.X - lastSampledPos.X;
                double dy = entity.Pos.Y - lastSampledPos.Y;
                double dz = entity.Pos.Z - lastSampledPos.Z;

                if (dx * dx + dy * dy + dz * dz < SampleThresholdSq)
                {
                    return;
                }

                lastSampledPos.Set(entity.Pos.X, entity.Pos.Y, entity.Pos.Z);

                UpdateSurfaceData();
            }
            catch (Exception e)
            {
                // Suppress transient geometry resolution errors to maintain simulation stability.
                entity.World.Logger.VerboseDebug($"[SlopeLib] Suppressed error in EntityBehaviorSlopeAware: {e.Message}");
            }
        }

        /// <summary>
        /// Core evaluation routine. Executes a two-pass algorithm:
        /// 1. Flattens proximal block collision boundaries into a contiguous buffer.
        /// 2. Iterates faces, applying occlusion culling and inverse-square proximity weighting to resolve a unified surface normal.
        /// </summary>
        private void UpdateSurfaceData()
        {
            sphereRadius = CollisionSphereSize / 2.0;

            // Establish the origin of the geometric sampling sphere.
            center.Set(
                entity.Pos.X,
                entity.Pos.Y + CollisionSphereYOffset,
                entity.Pos.Z
            );

            // Re-use pre-allocated block positions
            minPos.Set((int)Math.Floor(center.X - sphereRadius - 1.0), (int)Math.Floor(center.Y - sphereRadius - 1.0), (int)Math.Floor(center.Z - sphereRadius - 1.0));
            maxPos.Set((int)Math.Ceiling(center.X + sphereRadius + 1.0), (int)Math.Ceiling(center.Y + sphereRadius + 1.0), (int)Math.Ceiling(center.Z + sphereRadius + 1.0));

            // Reset loop counters
            avgNormX = 0;
            avgNormY = 0;
            avgNormZ = 0;
            closestNormX = 0;
            closestNormY = 1;
            closestNormZ = 0;
            minDistanceToEdge = 9999.0;
            facesSampled = 0;
            closestX = center.X;
            closestY = center.Y;
            closestZ = center.Z;
            localBoxCount = 0;

            // Phase 1: Populate the local collision bounds array via spatial traversal.
            entity.World.BlockAccessor.WalkBlocks(minPos, maxPos, walkBlocksDelegate);

            BlockFacing[] facings = BlockFacing.ALLFACES;
            IBlockAccessor ba = entity.World.BlockAccessor;
            int lastBx = -999, lastBy = -999, lastBz = -999;

            for (int b = 0; b < localBoxCount; b++)
            {
                ref FastAABB box = ref localBoxes[b];

                // Reset adjacency cache when transitioning to a new block coordinate.
                if (box.bx != lastBx || box.by != lastBy || box.bz != lastBz)
                {
                    for (int i = 0; i < 6; i++) neighborBlocksFetched[i] = false;
                    lastBx = box.bx;
                    lastBy = box.by;
                    lastBz = box.bz;
                }

                for (int j = 0; j < facings.Length; j++)
                {
                    BlockFacing facing = facings[j];

                    // Evaluate surface exposure. Concealed faces are culled to prevent internal geometry from skewing the normal.
                    bool isExposed = true;

                    if (facing == BlockFacing.UP && box.y2 - box.by >= 0.999)
                        isExposed = !GetNeighbor(BlockFacing.UP, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.DOWN.Index];
                    else if (facing == BlockFacing.DOWN && box.y1 - box.by <= 0.001)
                        isExposed = !GetNeighbor(BlockFacing.DOWN, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.UP.Index];
                    else if (facing == BlockFacing.NORTH && box.z1 - box.bz <= 0.001)
                        isExposed = !GetNeighbor(BlockFacing.NORTH, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.SOUTH.Index];
                    else if (facing == BlockFacing.SOUTH && box.z2 - box.bz >= 0.999)
                        isExposed = !GetNeighbor(BlockFacing.SOUTH, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.NORTH.Index];
                    else if (facing == BlockFacing.WEST && box.x1 - box.bx <= 0.001)
                        isExposed = !GetNeighbor(BlockFacing.WEST, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.EAST.Index];
                    else if (facing == BlockFacing.EAST && box.x2 - box.bx >= 0.999)
                        isExposed = !GetNeighbor(BlockFacing.EAST, box.bx, box.by, box.bz, ba).SideSolid[BlockFacing.WEST.Index];

                    if (!isExposed) continue;

                    // Project the sphere's center onto the plane defined by the current block face.
                    // Inline clamp limits point coordinates to the face boundaries without method call overhead.
                    double pX = center.X < box.x1 ? box.x1 : (center.X > box.x2 ? box.x2 : center.X);
                    double pY = center.Y < box.y1 ? box.y1 : (center.Y > box.y2 ? box.y2 : center.Y);
                    double pZ = center.Z < box.z1 ? box.z1 : (center.Z > box.z2 ? box.z2 : center.Z);

                    // Snap to the plane of the face
                    if (facing == BlockFacing.UP) pY = box.y2;
                    else if (facing == BlockFacing.DOWN) pY = box.y1;
                    else if (facing == BlockFacing.NORTH) pZ = box.z1;
                    else if (facing == BlockFacing.SOUTH) pZ = box.z2;
                    else if (facing == BlockFacing.WEST) pX = box.x1;
                    else if (facing == BlockFacing.EAST) pX = box.x2;

                    // Calculate directional vector and distance metrics for occlusion culling and weight distribution.
                    double dirX = center.X - pX;
                    double dirY = center.Y - pY;
                    double dirZ = center.Z - pZ;

                    // Discard faces oriented away from the sampling origin.
                    double dot = dirX * facing.Normali.X + dirY * facing.Normali.Y + dirZ * facing.Normali.Z;
                    if (dot <= 0.0001) continue;

                    double distToCenterSq = dirX * dirX + dirY * dirY + dirZ * dirZ;
                    double distToCenter = Math.Sqrt(distToCenterSq);

                    // Calculate the adjusted distance relative to the sphere's external boundary.
                    double distToEdge = distToCenter - sphereRadius;
                    if (distToEdge < 0.0) distToEdge = 0.0;
                    double distToEdgeSq = distToEdge * distToEdge;

                    // Execute line-of-sight occlusion validation against the pre-compiled AABB collection.
                    if (distToCenterSq > 0.01)
                    {
                        if (IsOccluded(center.X, center.Y, center.Z, pX, pY, pZ, b)) continue;
                    }

                    // Register the geometric anchor point closest to the sampling sphere.
                    if (distToEdge < minDistanceToEdge)
                    {
                        minDistanceToEdge = distToEdge;
                        closestX = pX;
                        closestY = pY;
                        closestZ = pZ;
                        closestNormX = facing.Normali.X;
                        closestNormY = facing.Normali.Y;
                        closestNormZ = facing.Normali.Z;
                    }

                    // Apply inverse-square proximity weighting to the geometric normal. 
                    // The constant offset prevents division by zero without requiring conditional branches.
                    double weight = 1.0 / (distToEdgeSq + 0.001);

                    avgNormX += facing.Normali.X * weight;
                    avgNormY += facing.Normali.Y * weight;
                    avgNormZ += facing.Normali.Z * weight;

                    facesSampled++;
                }
            }

            averageNormal.Set(avgNormX, avgNormY, avgNormZ);

            // Phase 2: Normalize the accumulated weights to establish the final interpolated surface plane.
            if (facesSampled > 0)
            {
                // Handle symmetrical cancellation (e.g., wedged between parallel walls) by falling back to the nearest absolute face normal.
                if (averageNormal.LengthSq() > 0.001)
                {
                    averageNormal.Normalize();
                    SurfaceNormal.Set(averageNormal.X, averageNormal.Y, averageNormal.Z);
                }
                else
                {
                    SurfaceNormal.Set(closestNormX, closestNormY, closestNormZ);
                }

                // Calculate the orthogonal distance from the sphere's center to the resolved tangential plane.
                double vecX = center.X - closestX;
                double vecY = center.Y - closestY;
                double vecZ = center.Z - closestZ;

                double distCenterToPlane = vecX * averageNormal.X + vecY * averageNormal.Y + vecZ * averageNormal.Z;

                // Adjust the final distance relative to the boundary of the collision sphere.
                DistanceToSurface = distCenterToPlane - sphereRadius;

                // Translate the origin point along the inverted normal to determine the exact surface coordinate.
                SurfacePoint.Set(
                    center.X - SurfaceNormal.X * distCenterToPlane,
                    center.Y - SurfaceNormal.Y * distCenterToPlane,
                    center.Z - SurfaceNormal.Z * distCenterToPlane
                );
            }
            else
            {
                // Default state for unconstrained airspace.
                SurfaceNormal.Set(0, 1, 0);
                DistanceToSurface = 9999.0;
                SurfacePoint.Set(center.X, center.Y - sphereRadius, center.Z);
            }
        }

        /// <summary>
        /// Spatial query callback. Flattens world-space collision boxes into a pre-allocated stack buffer 
        /// to decouple geometric evaluation from chunk map query latency.
        /// </summary>
        private void OnBlockWalked(Block block, int bx, int by, int bz)
        {
            if (block.Id == 0) return;

            tmpPos.Set(bx, by, bz);
            Cuboidf[] boxes = block.GetCollisionBoxes(entity.World.BlockAccessor, tmpPos);
            if (boxes == null || boxes.Length == 0) return;

            for (int i = 0; i < boxes.Length; i++)
            {
                // Enforce a hard capacity limit to prevent buffer overruns from excessively detailed block geometries.
                if (localBoxCount >= 512) break;
                Cuboidf box = boxes[i];
                localBoxes[localBoxCount].x1 = bx + box.X1;
                localBoxes[localBoxCount].y1 = by + box.Y1;
                localBoxes[localBoxCount].z1 = bz + box.Z1;
                localBoxes[localBoxCount].x2 = bx + box.X2;
                localBoxes[localBoxCount].y2 = by + box.Y2;
                localBoxes[localBoxCount].z2 = bz + box.Z2;
                localBoxes[localBoxCount].bx = bx;
                localBoxes[localBoxCount].by = by;
                localBoxes[localBoxCount].bz = bz;
                localBoxCount++;
            }
        }

        /// <summary>
        /// Evaluates line-of-sight occlusion using a custom Liang-Barsky intersection algorithm.
        /// Operates exclusively on the pre-fetched AABB array to guarantee zero memory allocations.
        /// </summary>
        /// <param name="startX">Ray origin X.</param>
        /// <param name="startY">Ray origin Y.</param>
        /// <param name="startZ">Ray origin Z.</param>
        /// <param name="endX">Ray target X.</param>
        /// <param name="endY">Ray target Y.</param>
        /// <param name="endZ">Ray target Z.</param>
        /// <param name="ignoreBoxIndex">The index of the bounding box owning the target face, which must be ignored.</param>
        /// <returns>True if an obstruction intersects the ray segment; otherwise, false.</returns>
        private bool IsOccluded(double startX, double startY, double startZ, double endX, double endY, double endZ, int ignoreBoxIndex)
        {
            double dirX = endX - startX;
            double dirY = endY - startY;
            double dirZ = endZ - startZ;

            // Constrain the maximum traversal parameter (t) to bypass coplanar face intersection inaccuracies.
            double maxT = 0.99;

            for (int i = 0; i < localBoxCount; i++)
            {
                if (i == ignoreBoxIndex) continue;

                ref FastAABB box = ref localBoxes[i];

                // One-dimensional rapid rejection tests to bypass full intersection logic.
                double minRx = dirX < 0 ? startX + dirX : startX;
                double maxRx = dirX > 0 ? startX + dirX : startX;
                if (maxRx < box.x1 || minRx > box.x2) continue;

                double minRy = dirY < 0 ? startY + dirY : startY;
                double maxRy = dirY > 0 ? startY + dirY : startY;
                if (maxRy < box.y1 || minRy > box.y2) continue;

                double minRz = dirZ < 0 ? startZ + dirZ : startZ;
                double maxRz = dirZ > 0 ? startZ + dirZ : startZ;
                if (maxRz < box.z1 || minRz > box.z2) continue;

                // Implementation of the Liang-Barsky / Slab intersection algorithm.
                double tmin = 0.0;
                double tmax = maxT;

                if (dirX > -0.000001 && dirX < 0.000001) { if (startX < box.x1 || startX > box.x2) continue; }
                else
                {
                    double ood = 1.0 / dirX;
                    double t1 = (box.x1 - startX) * ood;
                    double t2 = (box.x2 - startX) * ood;
                    if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
                    if (t1 > tmin) tmin = t1;
                    if (t2 < tmax) tmax = t2;
                    if (tmin > tmax) continue;
                }

                // Affirmative intersection registered within the valid line segment.
                return true;
            }
            return false;
        }

        private Block GetNeighbor(BlockFacing face, int bx, int by, int bz, IBlockAccessor ba)
        {
            int idx = face.Index;
            if (neighborBlocksFetched[idx]) return neighborBlocks[idx];
            neighborBlocks[idx] = ba.GetBlock(bx + face.Normali.X, by + face.Normali.Y, bz + face.Normali.Z);
            neighborBlocksFetched[idx] = true;
            return neighborBlocks[idx];
        }
    }
}