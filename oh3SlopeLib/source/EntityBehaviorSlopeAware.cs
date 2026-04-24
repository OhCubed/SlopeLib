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
        /// Shortest distance from the entity's center to the closest block collision box.
        /// </summary>
        public double DistanceToSurface { get; private set; } = 9999.0;

        /// <summary>
        /// If true, bypasses volumetric logic and uses a basic downward raycast.
        /// </summary>
        public bool UseFastPathFloorOnly = false;

        private Vec3d lastSampledPos = new Vec3d(0, -9999, 0);
        private const double SampleThresholdSq = 0.1 * 0.1;

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
        /// Core Algorithm: Averaged Exposed Faces
        /// Uses WalkBlocks to gather all collision boxes, culls internal hidden faces, and averages 
        /// the pure geometric normals of the exposed surfaces. This completely eliminates grid bias 
        /// while beautifully smoothing out stairs and voxel terrain into natural slopes!
        /// </summary>
        private void UpdateSurfaceData()
        {
            Cuboidf selBox = entity.SelectionBox ?? new Cuboidf(-0.5f, 0, -0.5f, 0.5f, 1f, 0.5f);

            // We evaluate from the center of the entity to prevent division-by-zero 
            // when standing exactly on a face, and to "see" walls around us equally.
            Vec3d center = new Vec3d(
                entity.Pos.X,
                entity.Pos.Y + selBox.YSize / 2.0,
                entity.Pos.Z
            );

            // Define the local grid area to sample (Entity footprint + 1 block margin)
            int minX = (int)Math.Floor(center.X - selBox.XSize / 2.0 - 1.0);
            int minY = (int)Math.Floor(center.Y - selBox.YSize / 2.0 - 1.0);
            int minZ = (int)Math.Floor(center.Z - selBox.ZSize / 2.0 - 1.0);

            int maxX = (int)Math.Ceiling(center.X + selBox.XSize / 2.0 + 1.0);
            int maxY = (int)Math.Ceiling(center.Y + selBox.YSize / 2.0 + 1.0);
            int maxZ = (int)Math.Ceiling(center.Z + selBox.ZSize / 2.0 + 1.0);

            BlockPos minPos = new BlockPos(minX, minY, minZ);
            BlockPos maxPos = new BlockPos(maxX, maxY, maxZ);

            Vec3d averageNormal = new Vec3d(0, 0, 0);
            double minDistanceSq = 9999.0;
            int facesSampled = 0;

            BlockFacing[] facings = BlockFacing.ALLFACES;

            // 1. Efficiently query the engine for all blocks within the 3D area
            entity.World.BlockAccessor.WalkBlocks(minPos, maxPos, (block, bx, by, bz) =>
            {
                if (block.Id == 0) return;

                Cuboidf[] boxes = block.GetCollisionBoxes(entity.World.BlockAccessor, new BlockPos(bx, by, bz));
                if (boxes == null || boxes.Length == 0) return;

                foreach (var box in boxes)
                {
                    double worldX1 = bx + box.X1; double worldY1 = by + box.Y1; double worldZ1 = bz + box.Z1;
                    double worldX2 = bx + box.X2; double worldY2 = by + box.Y2; double worldZ2 = bz + box.Z2;

                    foreach (BlockFacing facing in facings)
                    {
                        // 2. Cull internal faces! This is what entirely eliminates the South-East bias.
                        // If a block face is pressed against another solid block, it doesn't push the player.
                        bool isExposed = true;

                        if (facing == BlockFacing.UP && box.Y2 >= 0.999f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx, by + 1, bz).SideSolid[BlockFacing.DOWN.Index];
                        else if (facing == BlockFacing.DOWN && box.Y1 <= 0.001f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx, by - 1, bz).SideSolid[BlockFacing.UP.Index];
                        else if (facing == BlockFacing.NORTH && box.Z1 <= 0.001f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx, by, bz - 1).SideSolid[BlockFacing.SOUTH.Index];
                        else if (facing == BlockFacing.SOUTH && box.Z2 >= 0.999f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx, by, bz + 1).SideSolid[BlockFacing.NORTH.Index];
                        else if (facing == BlockFacing.WEST && box.X1 <= 0.001f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx - 1, by, bz).SideSolid[BlockFacing.EAST.Index];
                        else if (facing == BlockFacing.EAST && box.X2 >= 0.999f)
                            isExposed = !entity.World.BlockAccessor.GetBlock(bx + 1, by, bz).SideSolid[BlockFacing.WEST.Index];

                        if (!isExposed) continue;

                        // 3. Find the mathematically closest point on this specific flat face
                        double pX = GameMath.Clamp(center.X, worldX1, worldX2);
                        double pY = GameMath.Clamp(center.Y, worldY1, worldY2);
                        double pZ = GameMath.Clamp(center.Z, worldZ1, worldZ2);

                        // Snap to the plane of the face
                        if (facing == BlockFacing.UP) pY = worldY2;
                        else if (facing == BlockFacing.DOWN) pY = worldY1;
                        else if (facing == BlockFacing.NORTH) pZ = worldZ1;
                        else if (facing == BlockFacing.SOUTH) pZ = worldZ2;
                        else if (facing == BlockFacing.WEST) pX = worldX1;
                        else if (facing == BlockFacing.EAST) pX = worldX2;

                        // 4. Calculate distance and direction
                        double dirX = center.X - pX;
                        double dirY = center.Y - pY;
                        double dirZ = center.Z - pZ;

                        // 5. We only care about surfaces facing TOWARDS the player's center
                        // Dot product > 0 ensures we aren't looking at the backside of a polygon
                        double dot = dirX * facing.Normali.X + dirY * facing.Normali.Y + dirZ * facing.Normali.Z;
                        if (dot <= 0.0001) continue;

                        double distSq = dirX * dirX + dirY * dirY + dirZ * dirZ;
                        minDistanceSq = Math.Min(minDistanceSq, distSq);

                        // 6. Weight the *pure geometric normal* by proximity
                        // Cap the min distance to prevent infinity/massive spikes if clipping inside a block
                        double weight = 1.0 / Math.Max(0.01, distSq);

                        averageNormal.Add(
                            facing.Normali.X * weight,
                            facing.Normali.Y * weight,
                            facing.Normali.Z * weight
                        );
                        facesSampled++;
                    }
                }
            });

            // 7. Average them all out into a perfectly smooth plane
            if (facesSampled > 0 && averageNormal.LengthSq() > 0.001)
            {
                SurfaceNormal = averageNormal.Normalize();
                DistanceToSurface = Math.Max(0.0, Math.Sqrt(minDistanceSq) - selBox.YSize / 2.0);
            }
            else
            {
                // Fallback for open air
                SurfaceNormal = new Vec3d(0, 1, 0);
                DistanceToSurface = 9999.0;
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