using System;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Tracks the local grid to provide a continuously interpolated surface normal
    /// and distance metric for custom entity physics (rolling, sliding, etc).
    /// </summary>
    public class EntityBehaviorSlopeAware : EntityBehavior
    {
        /// <summary>
        /// The smoothly interpolated normal of the surface nearest to the entity.
        /// </summary>
        public Vec3d SurfaceNormal { get; private set; } = new Vec3d(0, 1, 0);

        /// <summary>
        /// Shortest distance from the edge of the collision sphere to the calculated plane.
        /// </summary>
        public double DistanceToSurface { get; private set; } = 9999.0;

        /// <summary>
        /// The effective diameter of the sphere used for surface sensing.
        /// </summary>
        public double CollisionSphereSize { get; set; } = 1.0;

        /// <summary>
        /// How far up from the entity's feet the center of the sphere is located.
        /// </summary>
        public double CollisionSphereYOffset { get; set; } = 0.5;

        /// <summary>
        /// The exact world coordinate on the plane directly perpendicular to the sphere's center.
        /// </summary>
        public Vec3d SurfacePoint { get; private set; } = new Vec3d(0, 0, 0);

        private Vec3d lastSampledPos = new Vec3d(0, -9999, 0);
        private const double SampleThresholdSq = 0.1 * 0.1;

        // --- Pre-allocated memory buffers to prevent Garbage Collection spikes ---
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

        // Buffers specifically for the WalkBlocks delegate to avoid closure allocations
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

        // Neighbor caching to prevent querying the chunkmap multiple times for complex blocks
        private Block[] neighborBlocks = new Block[6];
        private bool[] neighborBlocksFetched = new bool[6];

        public EntityBehaviorSlopeAware(Entity entity) : base(entity)
        {
            // Cache the delegate once during initialization 
            walkBlocksDelegate = OnBlockWalked;
        }

        public override string PropertyName() => "slopeaware";

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            JsonObject slopelibConfig = null;

            // 1. Try finding "slopelib" in the entity's global attributes (entity.json -> attributes -> slopelib)
            if (properties.Attributes != null && properties.Attributes.KeyExists("slopelib"))
            {
                slopelibConfig = properties.Attributes["slopelib"];
            }
            // 2. Try finding "slopelib" nested inside the behavior's attributes
            else if (attributes != null && attributes.KeyExists("slopelib"))
            {
                slopelibConfig = attributes["slopelib"];
            }
            // 3. Fall back to the behavior's attributes directly if no "slopelib" wrapper was used
            else if (attributes != null && (attributes.KeyExists("diameter") || attributes.KeyExists("yoffset")))
            {
                slopelibConfig = attributes;
            }

            // Apply parsed values or default fallbacks
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

            // Safety net for server lag / chunk loading edges
            if (entity?.World?.BlockAccessor == null) return;

            try
            {
                // Crucial Optimization: Calculate distance manually to avoid entity.Pos.XYZ allocating a new Vec3d
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
                // Graceful fallback if block data or chunk limits throw an unexpected error
                entity.World.Logger.VerboseDebug($"[SlopeLib] Suppressed error in EntityBehaviorSlopeAware: {e.Message}");
            }
        }

        /// <summary>
        /// Core Algorithm: Averaged Exposed Faces
        /// Uses WalkBlocks to gather all collision boxes, culls internal hidden faces, and averages 
        /// the pure geometric normals of the exposed surfaces. 
        /// </summary>
        private void UpdateSurfaceData()
        {
            sphereRadius = CollisionSphereSize / 2.0;

            // 'center' now represents the center of our virtual sphere
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

            // 1. Efficiently query the engine to capture all local collision boxes into a flat array
            entity.World.BlockAccessor.WalkBlocks(minPos, maxPos, walkBlocksDelegate);

            BlockFacing[] facings = BlockFacing.ALLFACES;
            IBlockAccessor ba = entity.World.BlockAccessor;
            int lastBx = -999, lastBy = -999, lastBz = -999;

            for (int b = 0; b < localBoxCount; b++)
            {
                ref FastAABB box = ref localBoxes[b];

                // Reset neighbor cache when switching to a new block coordinate
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

                    // 2. Cull internal faces! This is what entirely eliminates the South-East bias.
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

                    // 3. Find the mathematically closest point on this specific flat face relative to the sphere center
                    // Manual inline clamp to avoid method call overhead
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

                    // 4. Calculate direction and distance for culling and weighting.
                    double dirX = center.X - pX;
                    double dirY = center.Y - pY;
                    double dirZ = center.Z - pZ;

                    // 5. We only care about surfaces facing TOWARDS the sphere center
                    double dot = dirX * facing.Normali.X + dirY * facing.Normali.Y + dirZ * facing.Normali.Z;
                    if (dot <= 0.0001) continue;

                    double distToCenterSq = dirX * dirX + dirY * dirY + dirZ * dirZ;
                    double distToCenter = Math.Sqrt(distToCenterSq);

                    // Distance from the EDGE of the sphere to the geometry
                    double distToEdge = distToCenter - sphereRadius;
                    if (distToEdge < 0.0) distToEdge = 0.0;
                    double distToEdgeSq = distToEdge * distToEdge;

                    // 5.5. Ultra-fast Custom Occlusion Check: Line intersection against our pre-gathered local boxes
                    if (distToCenterSq > 0.01)
                    {
                        if (IsOccluded(center.X, center.Y, center.Z, pX, pY, pZ, b)) continue;
                    }

                    // Track the anchor point closest to the sphere's edge
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

                    // 6. Weight the *pure geometric normal* by proximity to the sphere's edge
                    // The +0.001 smoothly prevents divide-by-zero without requiring branching/Math.Max
                    double weight = 1.0 / (distToEdgeSq + 0.001);

                    avgNormX += facing.Normali.X * weight;
                    avgNormY += facing.Normali.Y * weight;
                    avgNormZ += facing.Normali.Z * weight;

                    facesSampled++;
                }
            }

            averageNormal.Set(avgNormX, avgNormY, avgNormZ);

            // 7. Average them all out into a perfectly smooth plane
            if (facesSampled > 0)
            {
                // If opposing faces perfectly cancelled each other out (e.g. wedged in a 1-wide hallway),
                // the length will be 0. Fallback to the absolute closest surface instead of "open air".
                if (averageNormal.LengthSq() > 0.001)
                {
                    averageNormal.Normalize();
                    SurfaceNormal.Set(averageNormal.X, averageNormal.Y, averageNormal.Z);
                }
                else
                {
                    SurfaceNormal.Set(closestNormX, closestNormY, closestNormZ);
                }

                // Calculate the true mathematical distance from the sphere center to the plane 
                // that sits perfectly flush with the closest intersected geometry point.
                double vecX = center.X - closestX;
                double vecY = center.Y - closestY;
                double vecZ = center.Z - closestZ;

                double distCenterToPlane = vecX * averageNormal.X + vecY * averageNormal.Y + vecZ * averageNormal.Z;

                // The distance from the edge of the sphere to the plane
                DistanceToSurface = distCenterToPlane - sphereRadius;

                // Project the sphere's center onto the plane to get our SurfacePoint
                SurfacePoint.Set(
                    center.X - SurfaceNormal.X * distCenterToPlane,
                    center.Y - SurfaceNormal.Y * distCenterToPlane,
                    center.Z - SurfaceNormal.Z * distCenterToPlane
                );
            }
            else
            {
                // Fallback for open air
                SurfaceNormal.Set(0, 1, 0);
                DistanceToSurface = 9999.0;
                SurfacePoint.Set(center.X, center.Y - sphereRadius, center.Z);
            }
        }

        /// <summary>
        /// The cached delegate method for WalkBlocks. 
        /// Flattens all world-space collision boxes into our extremely fast struct array.
        /// </summary>
        private void OnBlockWalked(Block block, int bx, int by, int bz)
        {
            if (block.Id == 0) return;

            tmpPos.Set(bx, by, bz);
            Cuboidf[] boxes = block.GetCollisionBoxes(entity.World.BlockAccessor, tmpPos);
            if (boxes == null || boxes.Length == 0) return;

            for (int i = 0; i < boxes.Length; i++)
            {
                if (localBoxCount >= 512) break; // Hard safety limit against insanely complex modded blocks
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
        /// A custom, completely allocation-free ray/AABB intersection test.
        /// Bypasses the Engine's heavy RayTraceForSelection chunk queries entirely!
        /// </summary>
        private bool IsOccluded(double startX, double startY, double startZ, double endX, double endY, double endZ, int ignoreBoxIndex)
        {
            double dirX = endX - startX;
            double dirY = endY - startY;
            double dirZ = endZ - startZ;

            // Shrink the target distance slightly to avoid hitting adjacent coplanar faces
            double maxT = 0.99;

            for (int i = 0; i < localBoxCount; i++)
            {
                if (i == ignoreBoxIndex) continue;

                ref FastAABB box = ref localBoxes[i];

                // Fast rejection bounding box test
                double minRx = dirX < 0 ? startX + dirX : startX;
                double maxRx = dirX > 0 ? startX + dirX : startX;
                if (maxRx < box.x1 || minRx > box.x2) continue;

                double minRy = dirY < 0 ? startY + dirY : startY;
                double maxRy = dirY > 0 ? startY + dirY : startY;
                if (maxRy < box.y1 || minRy > box.y2) continue;

                double minRz = dirZ < 0 ? startZ + dirZ : startZ;
                double maxRz = dirZ > 0 ? startZ + dirZ : startZ;
                if (maxRz < box.z1 || minRz > box.z2) continue;

                // Liang-Barsky / Slab method
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

                if (dirY > -0.000001 && dirY < 0.000001) { if (startY < box.y1 || startY > box.y2) continue; }
                else
                {
                    double ood = 1.0 / dirY;
                    double t1 = (box.y1 - startY) * ood;
                    double t2 = (box.y2 - startY) * ood;
                    if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
                    if (t1 > tmin) tmin = t1;
                    if (t2 < tmax) tmax = t2;
                    if (tmin > tmax) continue;
                }

                if (dirZ > -0.000001 && dirZ < 0.000001) { if (startZ < box.z1 || startZ > box.z2) continue; }
                else
                {
                    double ood = 1.0 / dirZ;
                    double t1 = (box.z1 - startZ) * ood;
                    double t2 = (box.z2 - startZ) * ood;
                    if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
                    if (t1 > tmin) tmin = t1;
                    if (t2 < tmax) tmax = t2;
                    if (tmin > tmax) continue;
                }

                // If we made it here, there is a valid intersection between t=0 and t=0.99
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