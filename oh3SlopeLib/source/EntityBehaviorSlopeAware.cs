using System;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Represents the mathematical evaluation of a single distinct surface plane detected near a collision sphere.
    /// </summary>
    public class TrackedSurface
    {
        /// <summary>
        /// The orthogonal normal vector of the evaluated surface plane.
        /// </summary>
        public Vec3d SurfaceNormal { get; set; } = new Vec3d(0, 1, 0);

        /// <summary>
        /// The shortest distance from the exterior edge of the collision sphere to this surface plane.
        /// </summary>
        public double DistanceToSurface { get; set; } = 9999.0;

        /// <summary>
        /// The exact coordinate in world space where the surface plane was detected.
        /// </summary>
        public Vec3d SurfacePoint { get; set; } = new Vec3d(0, 0, 0);
    }

    /// <summary>
    /// Holds the geometric configuration of a single virtual sampling sphere and its resulting surface evaluation data.
    /// </summary>
    public class SurfaceData
    {
        /// <summary>
        /// A pre-allocated array of the 4 closest, distinctly separate surface planes detected near this sphere.
        /// </summary>
        public TrackedSurface[] Surfaces { get; private set; }

        /// <summary>
        /// The physical diameter of the sampling sphere in blocks/meters.
        /// </summary>
        public double CollisionSphereSize { get; set; } = 1.0;

        /// <summary>
        /// The X-axis offset of the sphere relative to the entity's center, before yaw rotation is applied.
        /// </summary>
        public double CollisionSphereXOffset { get; set; } = 0.0;

        /// <summary>
        /// The Y-axis (vertical) offset of the sphere relative to the entity's center.
        /// </summary>
        public double CollisionSphereYOffset { get; set; } = 0.5;

        /// <summary>
        /// The Z-axis offset of the sphere relative to the entity's center, before yaw rotation is applied.
        /// </summary>
        public double CollisionSphereZOffset { get; set; } = 0.0;

        /// <summary>
        /// Initializes a new instance of the <see cref="SurfaceData"/> class and pre-allocates surface tracking memory.
        /// </summary>
        public SurfaceData()
        {
            Surfaces = new TrackedSurface[4];
            for (int i = 0; i < 4; i++) Surfaces[i] = new TrackedSurface();
        }
    }

    /// <summary>
    /// Extends entity physics by generating dynamically interpolated surface normals and distance metrics.
    /// Designed with strict zero-allocation (GC-free) hot paths for maximum runtime performance.
    /// </summary>
    public class EntityBehaviorSlopeAware : EntityBehavior
    {
        /// <summary>
        /// The collection of surface properties and geometric configurations tracked for this entity.
        /// </summary>
        public SurfaceData[] SurfaceDataList { get; set; } = new SurfaceData[] { new SurfaceData() };

        // --- Execution Control ---

        /// <summary>The entity's coordinate during the last successful surface evaluation.</summary>
        private Vec3d lastSampledPos = new Vec3d(0, -9999, 0);
        /// <summary>The entity's yaw rotation during the last successful surface evaluation.</summary>
        private double lastSampledYaw = -999.0;

        /// <summary>
        /// The squared distance the entity must move before triggering a new geometric evaluation. 
        /// Prevents unnecessary mathematical overhead when standing still.
        /// </summary>
        private const double SampleThresholdSq = 0.1 * 0.1;

        /// <summary>
        /// The angular delta (in radians) the entity must rotate before triggering a new geometric evaluation.
        /// </summary>
        private const double SampleYawThreshold = 0.05;

        // --- Error State Tracking ---
        private int consecutiveTickErrors = 0;
        private int consecutiveRaycastErrors = 0;

        // --- Pre-allocated Buffers (Zero-Allocation Physics Design) ---
        // These buffers prevent Garbage Collection (GC) spikes during the physics tick loop.
        private BlockPos minPos = new BlockPos();
        private BlockPos maxPos = new BlockPos();
        private BlockPos tmpPos = new BlockPos();
        private BlockPos raycastPos = new BlockPos();
        private Vec3d center = new Vec3d();
        private Vec3d averageNormal = new Vec3d();

        /// <summary>
        /// A lightweight, struct-based Axis-Aligned Bounding Box (AABB) representation 
        /// used to flatten proximal collision data for rapid, memory-contiguous traversal.
        /// </summary>
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
        private double closestX, closestY, closestZ;
        private double closestNormX, closestNormY, closestNormZ;
        private double avgNormX, avgNormY, avgNormZ;
        private Action<Block, int, int, int> walkBlocksDelegate;

        // Adjacency cache to minimize chunk dictionary lookup overhead during block iteration.
        private Block[] neighborBlocks = new Block[6];
        private bool[] neighborBlocksFetched = new bool[6];

        /// <summary>
        /// Tracks faces that served as the primary anchor in previous evaluation passes,
        /// ensuring each pass detects a distinctly separate geometric plane.
        /// </summary>
        private struct ExcludedFace
        {
            public double pX, pY, pZ;
            public int faceIndex;
        }
        private ExcludedFace[] excludedFaces = new ExcludedFace[4];

        /// <summary>
        /// Initializes a new instance of the <see cref="EntityBehaviorSlopeAware"/> class.
        /// </summary>
        /// <param name="entity">The entity this behavior is attached to.</param>
        public EntityBehaviorSlopeAware(Entity entity) : base(entity)
        {
            // Cache the delegate once during initialization to prevent implicit heap allocations on every frame.
            walkBlocksDelegate = OnBlockWalked;
        }

        /// <summary>
        /// The unique registry string identifier for this behavior.
        /// </summary>
        /// <returns>The string "slopeaware".</returns>
        public override string PropertyName() => "slopeaware";

        /// <summary>
        /// Called when the entity is initialized. Resolves configuration parameters from either global entity attributes or behavior-specific properties.
        /// </summary>
        /// <param name="properties">The entity's type properties.</param>
        /// <param name="attributes">The specific JSON attributes attached to this behavior instance.</param>
        /// <remarks>
        /// Supports both legacy single-sphere JSON syntax and the modern JSON array syntax for multi-sphere setups.
        /// </remarks>
        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (SurfaceDataList == null) SurfaceDataList = new SurfaceData[] { new SurfaceData() };

            JsonObject behaviorConfig = null;

            // Attempt to resolve configuration from the entity's global attributes.
            if (properties.Attributes != null && properties.Attributes.KeyExists("slopelib"))
            {
                behaviorConfig = properties.Attributes["slopelib"];
            }
            // Attempt to resolve configuration nested within the behavior's attributes.
            else if (attributes != null && attributes.KeyExists("slopelib"))
            {
                behaviorConfig = attributes["slopelib"];
            }
            // Fall back to the behavior's attributes directly if no "slopelib" wrapper was used.
            else if (attributes != null && (attributes.KeyExists("diameter") || attributes.KeyExists("yoffset")))
            {
                behaviorConfig = attributes;
            }

            // Apply resolved configuration or enforce default fallbacks.
            if (behaviorConfig != null)
            {
                JsonObject[] arr = behaviorConfig.AsArray();
                if (arr != null && arr.Length > 0 && !behaviorConfig.KeyExists("diameter"))
                {
                    SurfaceDataList = new SurfaceData[arr.Length];
                    for (int i = 0; i < arr.Length; i++)
                    {
                        SurfaceDataList[i] = new SurfaceData();
                        ParseSurfaceDataConfig(SurfaceDataList[i], arr[i]);
                    }
                }
                else
                {
                    SurfaceDataList = new SurfaceData[] { new SurfaceData() };
                    ParseSurfaceDataConfig(SurfaceDataList[0], behaviorConfig);
                }
            }
            else
            {
                SurfaceDataList = new SurfaceData[] { new SurfaceData() };
            }
        }

        /// <summary>
        /// Dynamically updates the geometric configuration of a specific sampling sphere without requiring memory reallocation.
        /// </summary>
        /// <param name="index">The array index of the sphere to update.</param>
        /// <param name="size">The new physical diameter of the sphere.</param>
        /// <param name="xOffset">The new X-axis local offset.</param>
        /// <param name="yOffset">The new Y-axis local offset.</param>
        /// <param name="zOffset">The new Z-axis local offset.</param>
        public void UpdateSphereConfig(int index, double size, double xOffset, double yOffset, double zOffset)
        {
            if (SurfaceDataList != null && index >= 0 && index < SurfaceDataList.Length)
            {
                SurfaceDataList[index].CollisionSphereSize = size;
                SurfaceDataList[index].CollisionSphereXOffset = xOffset;
                SurfaceDataList[index].CollisionSphereYOffset = yOffset;
                SurfaceDataList[index].CollisionSphereZOffset = zOffset;
            }
        }

        /// <summary>
        /// Applies JSON configuration values to a target <see cref="SurfaceData"/> object.
        /// </summary>
        /// <param name="data">The surface data object to populate.</param>
        /// <param name="config">The JSON object containing the properties.</param>
        private void ParseSurfaceDataConfig(SurfaceData data, JsonObject config)
        {
            data.CollisionSphereSize = config["diameter"]?.AsDouble(1.0) ?? 1.0;
            data.CollisionSphereXOffset = config["xoffset"]?.AsDouble(0.0) ?? 0.0;
            data.CollisionSphereYOffset = config["yoffset"]?.AsDouble(data.CollisionSphereSize / 2.0) ?? (data.CollisionSphereSize / 2.0);
            data.CollisionSphereZOffset = config["zoffset"]?.AsDouble(0.0) ?? 0.0;
        }

        /// <summary>
        /// Called during the main engine loop. Responsible for conditionally triggering terrain evaluation.
        /// </summary>
        /// <param name="deltaTime">The time elapsed since the last tick.</param>
        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            // Ensure the entity is fully loaded, possesses a position, and has valid configuration arrays
            if (entity?.World?.BlockAccessor == null || entity.Pos == null || SurfaceDataList == null) return;

            try
            {
                // Calculate movement delta manually to prevent implicit Vec3d allocations from property getters.
                double dx = entity.Pos.X - lastSampledPos.X;
                double dy = entity.Pos.Y - lastSampledPos.Y;
                double dz = entity.Pos.Z - lastSampledPos.Z;
                double dyaw = entity.Pos.Yaw - lastSampledYaw;

                // Optimization: Skip heavy intersection math if the entity hasn't translated or rotated significantly.
                if (dx * dx + dy * dy + dz * dz < SampleThresholdSq && Math.Abs(dyaw) < SampleYawThreshold)
                {
                    return;
                }

                lastSampledPos.Set(entity.Pos.X, entity.Pos.Y, entity.Pos.Z);
                lastSampledYaw = entity.Pos.Yaw;

                UpdateSurfaceData();

                // Clear the error throttle upon a successful frame completion
                consecutiveTickErrors = 0;
            }
            catch (Exception e)
            {
                // Throttle log output to prevent severe I/O lag from continuous log spam if geometry fails
                if (consecutiveTickErrors < 5)
                {
                    entity.World.Logger.VerboseDebug($"[SlopeLib] Suppressed error in EntityBehaviorSlopeAware (Tick): {e.Message}");
                    consecutiveTickErrors++;
                }
            }
        }

        /// <summary>
        /// Core evaluation routine. Executes a multiphase algorithm:
        /// 1. Determines the bounding volume covering all configured spheres.
        /// 2. Flattens proximal block collision boundaries within that volume into a contiguous struct buffer.
        /// 3. Iterates 4 passes per sphere, applying occlusion culling, proximity weighting, and coplanar 
        ///    exclusion to resolve the 4 nearest mathematically distinct surface planes.
        /// </summary>
        private void UpdateSurfaceData()
        {
            if (SurfaceDataList.Length == 0) return;

            // OPTIMIZATION: Cache the entity's precise location locally to prevent virtual property dispatch overhead in the loop
            double entX = entity.Pos.X;
            double entY = entity.Pos.Y;
            double entZ = entity.Pos.Z;
            double cosYaw = Math.Cos(entity.Pos.Yaw);
            double sinYaw = Math.Sin(entity.Pos.Yaw);

            // Phase 1: Establish the combined maximum bounds for spatial chunk queries across all configured spheres
            double minX = 999999, minY = 999999, minZ = 999999;
            double maxX = -999999, maxY = -999999, maxZ = -999999;

            for (int s = 0; s < SurfaceDataList.Length; s++)
            {
                var sd = SurfaceDataList[s];
                double sRad = sd.CollisionSphereSize / 2.0;

                // Translate the sphere's local offsets into world space based on the entity's yaw rotation
                double cX = entX + (sd.CollisionSphereXOffset * cosYaw - sd.CollisionSphereZOffset * sinYaw);
                double cY = entY + sd.CollisionSphereYOffset;
                double cZ = entZ + (sd.CollisionSphereXOffset * sinYaw + sd.CollisionSphereZOffset * cosYaw);

                if (cX - sRad - 1.0 < minX) minX = cX - sRad - 1.0;
                if (cY - sRad - 1.0 < minY) minY = cY - sRad - 1.0;
                if (cZ - sRad - 1.0 < minZ) minZ = cZ - sRad - 1.0;

                if (cX + sRad + 1.0 > maxX) maxX = cX + sRad + 1.0;
                if (cY + sRad + 1.0 > maxY) maxY = cY + sRad + 1.0;
                if (cZ + sRad + 1.0 > maxZ) maxZ = cZ + sRad + 1.0;
            }

            minPos.Set((int)Math.Floor(minX), (int)Math.Floor(minY), (int)Math.Floor(minZ));
            maxPos.Set((int)Math.Ceiling(maxX), (int)Math.Ceiling(maxY), (int)Math.Ceiling(maxZ));

            localBoxCount = 0;

            // Execute the world query strictly once, dumping all geometry into our local struct cache
            entity.World.BlockAccessor.WalkBlocks(minPos, maxPos, walkBlocksDelegate);

            BlockFacing[] facings = BlockFacing.ALLFACES;
            IBlockAccessor ba = entity.World.BlockAccessor;

            // Phase 2: Iterate and evaluate faces for each independent surface data object
            for (int s = 0; s < SurfaceDataList.Length; s++)
            {
                var sd = SurfaceDataList[s];
                sphereRadius = sd.CollisionSphereSize / 2.0;

                center.Set(
                    entX + (sd.CollisionSphereXOffset * cosYaw - sd.CollisionSphereZOffset * sinYaw),
                    entY + sd.CollisionSphereYOffset,
                    entZ + (sd.CollisionSphereXOffset * sinYaw + sd.CollisionSphereZOffset * cosYaw)
                );

                // Run 4 distinct evaluation passes to find the top 4 geometric layers
                for (int pass = 0; pass < 4; pass++)
                {
                    avgNormX = 0; avgNormY = 0; avgNormZ = 0;
                    closestNormX = 0; closestNormY = 1; closestNormZ = 0;
                    minDistanceToEdge = 9999.0;
                    facesSampled = 0;
                    closestX = center.X; closestY = center.Y; closestZ = center.Z;

                    double bestPx = 0, bestPy = 0, bestPz = 0;
                    int bestFaceIdx = -1;

                    int lastBx = -999, lastBy = -999, lastBz = -999;

                    for (int b = 0; b < localBoxCount; b++)
                    {
                        ref FastAABB box = ref localBoxes[b];

                        // Reset adjacency cache when transitioning to a new block coordinate.
                        if (box.bx != lastBx || box.by != lastBy || box.bz != lastBz)
                        {
                            for (int i = 0; i < 6; i++) neighborBlocksFetched[i] = false;
                            lastBx = box.bx; lastBy = box.by; lastBz = box.bz;
                        }

                        for (int j = 0; j < facings.Length; j++)
                        {
                            BlockFacing facing = facings[j];

                            bool isExposed = true;

                            // Evaluate surface exposure. Concealed interior faces are culled to prevent internal geometry from skewing the normal.
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

                            // Snap the projected point exactly to the plane of the face.
                            if (facing == BlockFacing.UP) pY = box.y2;
                            else if (facing == BlockFacing.DOWN) pY = box.y1;
                            else if (facing == BlockFacing.NORTH) pZ = box.z1;
                            else if (facing == BlockFacing.SOUTH) pZ = box.z2;
                            else if (facing == BlockFacing.WEST) pX = box.x1;
                            else if (facing == BlockFacing.EAST) pX = box.x2;

                            // Coplanar Exclusion: Skip this block face if it lies on the exact same continuous plane 
                            // (like a long wall) as an anchor from a previously completed pass.
                            bool isExcluded = false;
                            for (int ex = 0; ex < pass; ex++)
                            {
                                if (excludedFaces[ex].faceIndex == facing.Index)
                                {
                                    // Get the vector between the current point and the previously excluded anchor point
                                    double vecX = pX - excludedFaces[ex].pX;
                                    double vecY = pY - excludedFaces[ex].pY;
                                    double vecZ = pZ - excludedFaces[ex].pZ;

                                    // Project that vector against the normal. If the result is near zero, the points are coplanar.
                                    double dotPlane = Math.Abs(vecX * facing.Normali.X + vecY * facing.Normali.Y + vecZ * facing.Normali.Z);

                                    if (dotPlane < 0.01)
                                    {
                                        isExcluded = true;
                                        break;
                                    }
                                }
                            }
                            if (isExcluded) continue;

                            // Calculate directional vector and distance metrics for occlusion culling and weight distribution.
                            double dirX = center.X - pX;
                            double dirY = center.Y - pY;
                            double dirZ = center.Z - pZ;

                            // Discard faces oriented away from the sampling origin (backface culling).
                            double dot = dirX * facing.Normali.X + dirY * facing.Normali.Y + dirZ * facing.Normali.Z;
                            if (dot <= 0.0001) continue;

                            // Monotonicity Check: Skip faces whose calculated infinite plane distance is physically closer 
                            // to our sphere center than the previous closest tracked face.
                            // A tiny epsilon prevents floating-point inaccuracies from falsely culling equidistant symmetric faces.
                            if (pass > 0)
                            {
                                double prevDistCenter = sd.Surfaces[pass - 1].DistanceToSurface + sphereRadius;
                                if (Math.Abs(dot) < Math.Abs(prevDistCenter) - 0.0001)
                                {
                                    continue;
                                }
                            }

                            double distToCenterSq = dirX * dirX + dirY * dirY + dirZ * dirZ;
                            double distToCenter = Math.Sqrt(distToCenterSq);

                            // Calculate the adjusted distance relative to the sphere's external boundary.
                            double distToEdge = distToCenter - sphereRadius;
                            if (distToEdge < 0.0) distToEdge = 0.0;
                            double distToEdgeSq = distToEdge * distToEdge;

                            // Execute line-of-sight occlusion validation against the pre-compiled AABB collection.
                            // If geometry is obstructing our view of this face, ignore it.
                            if (distToCenterSq > 0.01)
                            {
                                if (IsOccluded(center.X, center.Y, center.Z, pX, pY, pZ, b)) continue;
                            }

                            // Register the geometric anchor point closest to the sampling sphere.
                            if (distToEdge < minDistanceToEdge)
                            {
                                minDistanceToEdge = distToEdge;
                                closestX = pX; closestY = pY; closestZ = pZ;
                                closestNormX = facing.Normali.X;
                                closestNormY = facing.Normali.Y;
                                closestNormZ = facing.Normali.Z;
                                bestPx = pX; bestPy = pY; bestPz = pZ; bestFaceIdx = facing.Index;
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

                    // Phase 3: Normalize the accumulated weights to establish the final interpolated surface plane.
                    if (facesSampled > 0)
                    {
                        // Handle symmetrical cancellation (e.g., wedged tightly between parallel walls) by falling back to the nearest absolute face normal.
                        if (averageNormal.LengthSq() > 0.001)
                        {
                            averageNormal.Normalize();
                            sd.Surfaces[pass].SurfaceNormal.Set(averageNormal.X, averageNormal.Y, averageNormal.Z);
                        }
                        else
                        {
                            sd.Surfaces[pass].SurfaceNormal.Set(closestNormX, closestNormY, closestNormZ);
                        }

                        // Calculate the orthogonal distance from the sphere's center to the resolved tangential plane.
                        double vecX = center.X - closestX;
                        double vecY = center.Y - closestY;
                        double vecZ = center.Z - closestZ;

                        double distCenterToPlane = vecX * sd.Surfaces[pass].SurfaceNormal.X + vecY * sd.Surfaces[pass].SurfaceNormal.Y + vecZ * sd.Surfaces[pass].SurfaceNormal.Z;

                        // Adjust the final distance relative to the boundary of the collision sphere.
                        sd.Surfaces[pass].DistanceToSurface = distCenterToPlane - sphereRadius;

                        // Translate the origin point along the inverted normal to determine the exact surface coordinate.
                        sd.Surfaces[pass].SurfacePoint.Set(
                            center.X - sd.Surfaces[pass].SurfaceNormal.X * distCenterToPlane,
                            center.Y - sd.Surfaces[pass].SurfaceNormal.Y * distCenterToPlane,
                            center.Z - sd.Surfaces[pass].SurfaceNormal.Z * distCenterToPlane
                        );

                        // Add the closest face to the exclusion list for the next pass
                        excludedFaces[pass].pX = bestPx;
                        excludedFaces[pass].pY = bestPy;
                        excludedFaces[pass].pZ = bestPz;
                        excludedFaces[pass].faceIndex = bestFaceIdx;
                    }
                    else
                    {
                        // Default fallback state for unconstrained airspace. Early-out remaining passes.
                        for (int r = pass; r < 4; r++)
                        {
                            sd.Surfaces[r].SurfaceNormal.Set(0, 1, 0);
                            sd.Surfaces[r].DistanceToSurface = 9999.0;
                            sd.Surfaces[r].SurfacePoint.Set(center.X, center.Y - sphereRadius, center.Z);
                        }
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Spatial query callback. Flattens world-space collision boxes into a pre-allocated stack buffer 
        /// to decouple geometric evaluation from chunk map query latency.
        /// </summary>
        /// <param name="block">The block found at the coordinates.</param>
        /// <param name="bx">Block X position.</param>
        /// <param name="by">Block Y position.</param>
        /// <param name="bz">Block Z position.</param>
        private void OnBlockWalked(Block block, int bx, int by, int bz)
        {
            if (block.Id == 0) return;

            tmpPos.Set(bx, by, bz);
            Cuboidf[] boxes = block.GetCollisionBoxes(entity.World.BlockAccessor, tmpPos);
            if (boxes == null || boxes.Length == 0) return;

            for (int i = 0; i < boxes.Length; i++)
            {
                // Enforce a hard capacity limit to prevent buffer overruns from excessively detailed block geometries.
                // Using .Length ensures strict synchronization with array capacity bounds.
                if (localBoxCount >= localBoxes.Length) break;

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

                if (dirX > -0.000001 && dirX < 0.000001)
                {
                    if (startX < box.x1 || startX > box.x2) continue;
                }
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

        /// <summary>
        /// Retrieves an adjacent block utilizing a specialized local cache to minimize chunk dictionary lookups.
        /// </summary>
        /// <param name="face">The direction of the adjacent block relative to the origin block.</param>
        /// <param name="bx">The origin block X coordinate.</param>
        /// <param name="by">The origin block Y coordinate.</param>
        /// <param name="bz">The origin block Z coordinate.</param>
        /// <param name="ba">The active block accessor.</param>
        /// <returns>The block instance residing at the requested neighboring coordinate.</returns>
        private Block GetNeighbor(BlockFacing face, int bx, int by, int bz, IBlockAccessor ba)
        {
            int idx = face.Index;
            if (neighborBlocksFetched[idx]) return neighborBlocks[idx];
            neighborBlocks[idx] = ba.GetBlock(bx + face.Normali.X, by + face.Normali.Y, bz + face.Normali.Z);
            neighborBlocksFetched[idx] = true;
            return neighborBlocks[idx];
        }

        /// <summary>
        /// Emulates a zero-allocation raycast along the inverted surface normal to find the physical block 
        /// backing the mathematical plane.
        /// </summary>
        /// <param name="blockAccessor">The world block accessor.</param>
        /// <param name="normal">The evaluated normal vector to cast against.</param>
        /// <param name="pX">The origin X coordinate.</param>
        /// <param name="pY">The origin Y coordinate.</param>
        /// <param name="pZ">The origin Z coordinate.</param>
        /// <param name="maxDepth">The maximum physical penetration depth of the raycast.</param>
        /// <returns>The physical backing Block, or null if unresolvable.</returns>
        /// <remarks>Steps into the plane mathematically in fixed increments to ensure proper hit detection on partial blocks.</remarks>
        public Block GetPhysicalSurfaceBlock(IBlockAccessor blockAccessor, Vec3d normal, double pX, double pY, double pZ, double maxDepth = 0.8)
        {
            if (normal == null || blockAccessor == null) return null;

            try
            {
                // Step into the plane mathematically in fixed increments.
                for (double d = 0.2; d <= maxDepth + 0.01; d += 0.2)
                {
                    double cX = pX - normal.X * d;
                    double cY = pY - normal.Y * d;
                    double cZ = pZ - normal.Z * d;

                    raycastPos.Set(
                        (int)Math.Floor(cX),
                        (int)Math.Floor(cY),
                        (int)Math.Floor(cZ)
                    );

                    Block block = blockAccessor.GetBlock(raycastPos);

                    if (block == null || block.Id == 0) continue;

                    // Ensure the target block actually contains active collision bounds.
                    if (block.CollisionBoxes != null && block.CollisionBoxes.Length > 0)
                    {
                        bool intersects = false;

                        // Calculate local coordinates within the block's grid space to verify partial blocks (e.g., slabs).
                        double localX = cX - raycastPos.X;
                        double localY = cY - raycastPos.Y;
                        double localZ = cZ - raycastPos.Z;

                        // Iterate and verify the raycast actually hits the active bounds of partial blocks.
                        for (int i = 0; i < block.CollisionBoxes.Length; i++)
                        {
                            Cuboidf box = block.CollisionBoxes[i];
                            if (localX >= box.X1 && localX <= box.X2 &&
                                localY >= box.Y1 && localY <= box.Y2 &&
                                localZ >= box.Z1 && localZ <= box.Z2)
                            {
                                intersects = true;
                                break;
                            }
                        }

                        if (intersects)
                        {
                            // Defer to the unified utility for Microblock extraction, returning the true backing material.
                            return MaterialUtility.GetMaterialBlock(blockAccessor, block, raycastPos);
                        }
                    }
                }

                // Clear error throttle upon finding no issues or successfully exiting the function
                consecutiveRaycastErrors = 0;
            }
            catch (Exception e)
            {
                // Throttle log output to prevent severe I/O lag from continuous log spam if geometry fails
                if (consecutiveRaycastErrors < 5)
                {
                    entity?.World?.Logger.VerboseDebug($"[SlopeLib] Suppressed error during surface block raycast: {e.Message}");
                    consecutiveRaycastErrors++;
                }
            }

            return null;
        }
    }
}