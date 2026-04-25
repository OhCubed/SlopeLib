using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;
using System;

namespace oh3SlopeLib
{
    /// <summary>
    /// Handles the visual rendering of debug information for the slope engine,
    /// such as surface normals and sampling radiuses.
    /// </summary>
    public class SlopeDebugRenderer : IRenderer
    {
        private ICoreClientAPI capi;
        private oh3SlopeLibModSystem modSys;
        private MeshRef cachedPlaneMeshRef;

        // --- Pre-allocated memory buffers to prevent Garbage Collection spikes ---
        private float[] modelMatrix = Mat4f.Create();
        private float[] rotationAxis = new float[3];
        private Vec4f overrideLight = new Vec4f(1f, 1f, 1f, 1f);
        private Vec3f overrideAmbient = new Vec3f(1f, 1f, 1f);

        /// <summary>
        /// Controls whether the debug visuals are actively drawn.
        /// </summary>
        public bool IsActive { get; set; } = false;

        public SlopeDebugRenderer(ICoreClientAPI capi, oh3SlopeLibModSystem modSys)
        {
            this.capi = capi;
            this.modSys = modSys;
            BuildGlowingPlaneMesh(1.5f);
        }

        /// <summary>
        /// 1. Initialization (Build the Mesh Once)
        /// </summary>
        private void BuildGlowingPlaneMesh(float size)
        {
            // Capacity: 6 vertices, 10 indices. Standard shader requires UVs, Normals, and Flags.
            MeshData mesh = new MeshData(6, 10, true, true, true, true);
            mesh.SetMode(EnumDrawMode.Lines);

            // Vertices: 4 form a flat quad on the X/Z axis at Y=0, 1 sits at (0, size, 0), 1 sits at center (0, 0, 0)
            mesh.xyz = new float[] {
                -0.5f, 0, -0.5f, // 0
                 0.5f, 0, -0.5f, // 1
                 0.5f, 0,  0.5f, // 2
                -0.5f, 0,  0.5f, // 3
                 0, size, 0,     // 4 (Tip)
                 0, 0, 0         // 5 (Center)
            };
            mesh.VerticesCount = 6;

            mesh.Uv = new float[12];
            mesh.Normals = new int[6];
            mesh.Flags = new int[6];

            // Indices: 8 to draw the perimeter of the quad, 2 to draw the line from center to tip
            mesh.Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                5, 4
            };
            mesh.IndicesCount = 10;

            // Standard shader expects the mesh to be white, and tinted via RgbaTint later
            mesh.Rgba = new byte[24];
            for (int i = 0; i < 6; i++)
            {
                mesh.Rgba[i * 4 + 0] = 255;
                mesh.Rgba[i * 4 + 1] = 255;
                mesh.Rgba[i * 4 + 2] = 255;
                mesh.Rgba[i * 4 + 3] = 255;
                mesh.Uv[i * 2 + 0] = 0.5f;
                mesh.Uv[i * 2 + 1] = 0.5f;
                mesh.Normals[i] = 0;
                mesh.Flags[i] = 0;
            }

            cachedPlaneMeshRef = capi.Render.UploadMesh(mesh);
        }

        /// <summary>
        /// Determines the rendering order. Debug info usually renders in the Opaque or Shadows stage.
        /// </summary>
        public double RenderOrder => 0.5;

        /// <summary>
        /// Required by IRenderer. Sets the render distance limit.
        /// </summary>
        public int RenderRange => 24;

        /// <summary>
        /// Determines which render stage this renderer should execute in.
        /// </summary>
        public EnumRenderStage Stage => EnumRenderStage.Opaque;

        public void OnRenderFrame(float deltaTime, EnumRenderStage stage)
        {
            // Abort early if the debug view is toggled off, or if the player/world is currently loading/unloading
            if (!IsActive || cachedPlaneMeshRef == null || capi?.World?.Player?.Entity == null) return;

            // Use StandardShader matching the working example
            IStandardShaderProgram prog = capi.Render.StandardShader;
            if (prog == null) return;

            prog.Use();
            prog.DontWarpVertices = 1;

            // Mimic the exact lighting and coloring logic that worked in the previous mod
            prog.RgbaTint = modSys.DebugColorVec;
            prog.RgbaLightIn = overrideLight;
            prog.RgbaAmbientIn = overrideAmbient;
            prog.ExtraGlow = 255;
            prog.ProjectionMatrix = capi.Render.CurrentProjectionMatrix;
            prog.ViewMatrix = capi.Render.CameraMatrixOriginf;

            Vec3d camPos = capi.World.Player.Entity.CameraPos;

            // CRITICAL OPTIMIZATION: Cache property getters outside the loop 
            // so we aren't crossing into engine memory for every entity.
            double camX = camPos.X;
            double camY = camPos.Y;
            double camZ = camPos.Z;

            // Wrap the rendering in a try-catch to guarantee we NEVER crash the main game thread
            try
            {
                // CRITICAL OPTIMIZATION: Iterate KeyValuePairs directly. Calling .Values on a ConcurrentDictionary 
                // allocates a new array and ReadOnlyCollection every single frame, causing massive GC stutters!
                foreach (var kvp in capi.World.LoadedEntities)
                {
                    var entity = kvp.Value;

                    // Skip null, dead, or despawning entities
                    if (entity == null || !entity.Alive) continue;

                    var tracker = entity.GetBehavior<EntityBehaviorSlopeAware>();
                    if (tracker == null) continue;

                    // 1. Get data from our tracker behavior
                    Vec3d normal = tracker.SurfaceNormal;
                    Vec3d pos = tracker.SurfacePoint; // Use the exact surface intersection point

                    // 2. Highly optimized cross product and dot product assuming 'Up' is always exactly (0, 1, 0)
                    // This avoids allocating Vec3f objects or arrays every frame
                    float nX = (float)normal.X;
                    float nY = (float)normal.Y;
                    float nZ = (float)normal.Z;

                    // 3. Render Setup - Re-use the existing model matrix buffer and manually apply Identity + Translation.
                    // This completely bypasses the Mat4f.Identity() and Mat4f.Translate() method invocation overhead!
                    modelMatrix[0] = 1f; modelMatrix[1] = 0f; modelMatrix[2] = 0f; modelMatrix[3] = 0f;
                    modelMatrix[4] = 0f; modelMatrix[5] = 1f; modelMatrix[6] = 0f; modelMatrix[7] = 0f;
                    modelMatrix[8] = 0f; modelMatrix[9] = 0f; modelMatrix[10] = 1f; modelMatrix[11] = 0f;

                    // 4. Position relative to camera
                    modelMatrix[12] = (float)(pos.X - camX);
                    modelMatrix[13] = (float)(pos.Y - camY);
                    modelMatrix[14] = (float)(pos.Z - camZ);
                    modelMatrix[15] = 1f;

                    // Calculate rotation efficiently using locals instead of array lookups
                    float lenSq = nZ * nZ + nX * nX;
                    float len = lenSq > 0.000001f ? (float)Math.Sqrt(lenSq) : 0f;

                    // Apply the calculated rotation to our custom ModelMatrix safely
                    if (len > 0.0001f)
                    {
                        rotationAxis[0] = nZ / len;
                        rotationAxis[1] = 0;
                        rotationAxis[2] = -nX / len;

                        // Manual inline clamp to bypass GameMath method overhead
                        float dot = nY < -1f ? -1f : (nY > 1f ? 1f : nY);
                        float angle = (float)Math.Acos(dot);

                        Mat4f.Rotate(modelMatrix, modelMatrix, angle, rotationAxis);
                    }
                    else if (nY < -0.999f)
                    {
                        // Fallback only if pointing exactly straight down (nY == -1)
                        rotationAxis[0] = 1;
                        rotationAxis[1] = 0;
                        rotationAxis[2] = 0;
                        Mat4f.Rotate(modelMatrix, modelMatrix, (float)Math.PI, rotationAxis);
                    }

                    // 5. Draw and Cleanup
                    prog.ModelMatrix = modelMatrix;

                    capi.Render.RenderMesh(cachedPlaneMeshRef);
                }
            }
            catch (Exception e)
            {
                // Graceful fallback if math or matrices ever go completely invalid
                capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error during debug render: {e.Message}");
            }

            prog.Stop();
        }

        public void Dispose()
        {
            cachedPlaneMeshRef?.Dispose();
        }
    }
}