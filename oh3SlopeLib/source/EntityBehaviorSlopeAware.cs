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
        /// The smoothly interpolated normal of the surface below the entity.
        /// </summary>
        public Vec3d SurfaceNormal { get; private set; } = new Vec3d(0, 1, 0);

        /// <summary>
        /// Shortest distance from the entity's center to the closest block collision box.
        /// </summary>
        public double DistanceToSurface { get; private set; } = 9999.0;

        /// <summary>
        /// If true, bypasses WalkBlocks and uses a basic raycast to determine the floor.
        /// </summary>
        public bool UseFastPathFloorOnly = false;

        private Vec3d lastSampledPos = new Vec3d(0, -9999, 0);
        private const double SampleThresholdSq = 0.1 * 0.1;
        private const double SampleRadius = 1.5;

        public EntityBehaviorSlopeAware(Entity entity) : base(entity)
        {
        }

        public override string PropertyName() => "slopeaware";

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (attributes != null)
            {
                UseFastPathFloorOnly = attributes.IsTrue("useFastPathFloorOnly");
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            // Crucial Optimization: Skip expensive math if the entity hasn't moved much (Jitter reduction)
            if (entity.Pos.XYZ.SquareDistanceTo(lastSampledPos) < SampleThresholdSq)
            {
                return;
            }

            lastSampledPos.Set(entity.Pos.X, entity.Pos.Y, entity.Pos.Z);

            if (UseFastPathFloorOnly)
            {
                UpdateSurfaceDataFastPath();
            }
            else
            {
                UpdateSurfaceData();
            }
        }

        /// <summary>
        /// Core Algorithm: Distance-Weighted Normals
        /// Samples the local 3D grid and builds a continuous surface normal from nearby collision boxes.
        /// </summary>
        private void UpdateSurfaceData()
        {
            Cuboidf selBox = entity.SelectionBox ?? new Cuboidf(-0.5f, 0, -0.5f, 0.5f, 1f, 0.5f);

            // Focus our sampling center slightly above the bottom of the bounding box
            Vec3d center = entity.Pos.XYZ.Clone();
            center.Y += selBox.Y2 / 2.0;

            // Expand a sample boundary box
            Cuboidd bounds = selBox.ToDouble().Translate(entity.Pos.XYZ);
            bounds.GrowBy(SampleRadius, SampleRadius, SampleRadius);

            BlockPos minPos = new BlockPos((int)Math.Floor(bounds.X1), (int)Math.Floor(bounds.Y1), (int)Math.Floor(bounds.Z1));
            BlockPos maxPos = new BlockPos((int)Math.Ceiling(bounds.X2), (int)Math.Ceiling(bounds.Y2), (int)Math.Ceiling(bounds.Z2));

            Vec3d normalSum = new Vec3d();
            double closestDist = 9999.0;

            // Highly optimized iteration through local grid
            entity.World.BlockAccessor.WalkBlocks(minPos, maxPos, (block, x, y, z) =>
            {
                if (block.Id == 0) return;

                Cuboidf[] collisionBoxes = block.GetCollisionBoxes(entity.World.BlockAccessor, new BlockPos(x, y, z));
                if (collisionBoxes == null || collisionBoxes.Length == 0) return;

                foreach (var box in collisionBoxes)
                {
                    Cuboidd worldBox = box.ToDouble().Translate(x, y, z);

                    // Find the mathematically closest point on this block's collision box to the entity's center
                    double cx = GameMath.Clamp(center.X, worldBox.X1, worldBox.X2);
                    double cy = GameMath.Clamp(center.Y, worldBox.Y1, worldBox.Y2);
                    double cz = GameMath.Clamp(center.Z, worldBox.Z1, worldBox.Z2);

                    // Create a normal vector pointing from the closest surface point back to the entity
                    Vec3d pointToCenter = new Vec3d(center.X - cx, center.Y - cy, center.Z - cz);
                    double dist = pointToCenter.Length();

                    closestDist = Math.Min(closestDist, dist);

                    if (dist > 0.001)
                    {
                        // Weighting: Inverse distance (Closer blocks pull the normal more aggressively)
                        double weight = 1.0 / dist;
                        pointToCenter.Normalize();
                        normalSum.Add(pointToCenter.X * weight, pointToCenter.Y * weight, pointToCenter.Z * weight);
                    }
                    else
                    {
                        // The entity center is exactly inside or perfectly on the collision box bounds
                        // Push up strongly to correct the overlap
                        normalSum.Add(0, 1000.0, 0);
                    }
                }
            });

            DistanceToSurface = closestDist;

            if (normalSum.LengthSq() > 0)
            {
                // Smoothly combines the weighted influence of all nearby collision steps
                SurfaceNormal = normalSum.Normalize();
            }
            else
            {
                // Default to a flat plane if floating far away from everything
                SurfaceNormal = new Vec3d(0, 1, 0);
            }
        }

        /// <summary>
        /// Fast-Path Alternative: Simplified single-ray downward cast for vehicles only requiring floor distance/face normal.
        /// </summary>
        private void UpdateSurfaceDataFastPath()
        {
            BlockSelection blockSel = new BlockSelection();
            EntitySelection entitySel = new EntitySelection();
            Vec3d pos = entity.Pos.XYZ;

            // Simple ray trace straight down
            entity.World.RayTraceForSelection(pos, pos.AddCopy(0, -2, 0), ref blockSel, ref entitySel);

            if (blockSel != null && blockSel.Position != null)
            {
                DistanceToSurface = pos.Y - blockSel.HitPosition.Y;
                SurfaceNormal = new Vec3d(blockSel.Face.Normali.X, blockSel.Face.Normali.Y, blockSel.Face.Normali.Z);
            }
            else
            {
                DistanceToSurface = 9999.0;
                SurfaceNormal = new Vec3d(0, 1, 0);
            }
        }
    }
}