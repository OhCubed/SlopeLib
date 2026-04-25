using System;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Handles the visual rendering of debug information for the slope engine.
    /// Draws oriented planes representing the calculated surface normal and distance for tracked entities.
    /// </summary>
    public class SlopeDebugRenderer : IRenderer
    {
        private ICoreClientAPI capi;
        private oh3SlopeLibModSystem modSys;
        private MeshRef cachedPlaneMeshRef;

        // --- Pre-allocated memory buffers to prevent Garbage Collection spikes in the render loop ---
        private float[] modelMatrix = Mat4f.Create();
        private float[] rotationAxis = new float[3];
        private Vec4f overrideLight = new Vec4f(1f, 1f, 1f, 1f);
        private Vec3f overrideAmbient = new Vec3f(1f, 1f, 1f);

        /// <summary>
        /// Gets or sets a value indicating whether the debug visuals should be rendered this frame.
        /// </summary>
        public bool IsActive { get; set; } = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="SlopeDebugRenderer"/> class.
        /// </summary>
        /// <param name="capi">The core client API.</param>
        /// <param name="modSys">The parent mod system utilized for configuration fetching.</param>
        public SlopeDebugRenderer(ICoreClientAPI capi, oh3SlopeLibModSystem modSys)
        {
            this.capi = capi;
            this.modSys = modSys;
            BuildGlowingPlaneMesh(1.5f);
        }

        /// <summary>
        /// Constructs the geometry for the debug plane mesh. Allocated once during initialization.
        /// </summary>
        /// <param name="size">The length of the directional normal indicator.</param>
        private void BuildGlowingPlaneMesh(float size)
        {
            // Capacity: 6 vertices, 10 indices. The standard shader requires UVs, Normals, and Flags.
            MeshData mesh = new MeshData(6, 10, true, true, true, true);
            mesh.SetMode(EnumDrawMode.Lines);

            // Define a flat quad centered at the origin, accompanied by a directional line pointing upwards.
            mesh.xyz = new float[] {
                -0.5f, 0, -0.5f, // 0: Bottom-left
                 0.5f, 0, -0.5f, // 1: Bottom-right
                 0.5f, 0,  0.5f, // 2: Top-right
                -0.5f, 0,  0.5f, // 3: Top-left
                 0, size, 0,     // 4: Normal indicator tip
                 0, 0, 0         // 5: Normal indicator base (Origin)
            };
            mesh.VerticesCount = 6;

            mesh.Uv = new float[12];
            mesh.Normals = new int[6];
            mesh.Flags = new int[6];

            // Specify vertex indices: 8 for the perimeter quad, 2 for the directional normal line.
            mesh.Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                5, 4
            };
            mesh.IndicesCount = 10;

            // Initialize default color (white), UV coordinates, and vertex flags. 
            // The shader will apply the dynamic configuration tint later.
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
        /// Defines the execution order relative to other renderers. 0.5 designates standard priority.
        /// </summary>
        public double RenderOrder => 0.5;

        /// <summary>
        /// Defines the maximum distance in blocks at which this renderer will execute.
        /// </summary>
        public int RenderRange => 24;

        /// <summary>
        /// Associates this renderer with the standard opaque rendering pass.
        /// </summary>
        public EnumRenderStage Stage => EnumRenderStage.Opaque;

        /// <summary>
        /// Executed by the engine every frame during the designated render stage.
        /// </summary>
        public void OnRenderFrame(float deltaTime, EnumRenderStage stage)
        {
            // Terminate early if debug mode is disabled, or if the client is currently undergoing a load/unload phase.
            if (!IsActive || cachedPlaneMeshRef == null || capi?.World?.Player?.Entity == null) return;

            // Utilize the engine's standard shader for basic unlit/tinted rendering.
            IStandardShaderProgram prog = capi.Render.StandardShader;
            if (prog == null) return;

            prog.Use();
            prog.DontWarpVertices = 1;

            // Apply debug coloring and enforce maximum glow to ensure the plane is visible in all lighting conditions.
            prog.RgbaTint = modSys.DebugColorVec;
            prog.RgbaLightIn = overrideLight;
            prog.RgbaAmbientIn = overrideAmbient;
            prog.ExtraGlow = 255;
            prog.ProjectionMatrix = capi.Render.CurrentProjectionMatrix;
            prog.ViewMatrix = capi.Render.CameraMatrixOriginf;

            Vec3d camPos = capi.World.Player.Entity.CameraPos;

            // OPTIMIZATION: Cache the camera position locally to prevent crossing into engine 
            // properties and allocating vectors for every tracked entity inside the inner loop.
            double camX = camPos.X;
            double camY = camPos.Y;
            double camZ = camPos.Z;

            // Wrap rendering logic in a try-catch block to prevent localized math exceptions 
            // from cascading and disrupting the client render thread.
            try
            {
                // OPTIMIZATION: Iterate the ConcurrentDictionary via KeyValuePair. Calling .Values 
                // dynamically allocates a new array and ReadOnlyCollection, which causes severe GC stutters.
                foreach (var kvp in capi.World.LoadedEntities)
                {
                    var entity = kvp.Value;

                    if (entity == null || !entity.Alive) continue;

                    var tracker = entity.GetBehavior<EntityBehaviorSlopeAware>();
                    if (tracker == null) continue;

                    // Retrieve calculated surface data from the slope behavior.
                    Vec3d normal = tracker.SurfaceNormal;
                    Vec3d pos = tracker.SurfacePoint;

                    // OPTIMIZATION: Cast components explicitly to avoid creating temporary Vec3f objects.
                    float nX = (float)normal.X;
                    float nY = (float)normal.Y;
                    float nZ = (float)normal.Z;

                    // Initialize the pre-allocated model matrix buffer with a pristine identity state.
                    // This manual assignment bypasses the method invocation overhead of Mat4f.Identity().
                    modelMatrix[0] = 1f; modelMatrix[1] = 0f; modelMatrix[2] = 0f; modelMatrix[3] = 0f;
                    modelMatrix[4] = 0f; modelMatrix[5] = 1f; modelMatrix[6] = 0f; modelMatrix[7] = 0f;
                    modelMatrix[8] = 0f; modelMatrix[9] = 0f; modelMatrix[10] = 1f; modelMatrix[11] = 0f;

                    // Apply translation relative to the camera's current viewport.
                    modelMatrix[12] = (float)(pos.X - camX);
                    modelMatrix[13] = (float)(pos.Y - camY);
                    modelMatrix[14] = (float)(pos.Z - camZ);
                    modelMatrix[15] = 1f;

                    // Determine the axis of rotation using an optimized, inline cross-product.
                    // This assumes the global 'Up' vector is constantly (0, 1, 0).
                    float lenSq = nZ * nZ + nX * nX;
                    float len = lenSq > 0.000001f ? (float)Math.Sqrt(lenSq) : 0f;

                    if (len > 0.0001f)
                    {
                        rotationAxis[0] = nZ / len;
                        rotationAxis[1] = 0;
                        rotationAxis[2] = -nX / len;

                        // Inline clamp to mathematically guarantee the dot product rests between -1 and 1.
                        // This entirely removes the risk of Math.Acos returning a NaN.
                        float dot = nY < -1f ? -1f : (nY > 1f ? 1f : nY);
                        float angle = (float)Math.Acos(dot);

                        Mat4f.Rotate(modelMatrix, modelMatrix, angle, rotationAxis);
                    }
                    else if (nY < -0.999f)
                    {
                        // Handle the edge case where the surface normal points exactly downwards.
                        rotationAxis[0] = 1;
                        rotationAxis[1] = 0;
                        rotationAxis[2] = 0;
                        Mat4f.Rotate(modelMatrix, modelMatrix, (float)Math.PI, rotationAxis);
                    }

                    // Upload the final transformation matrix to the shader and execute the draw call.
                    prog.ModelMatrix = modelMatrix;
                    capi.Render.RenderMesh(cachedPlaneMeshRef);
                }
            }
            catch (Exception e)
            {
                capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error during debug render: {e.Message}");
            }

            prog.Stop();
        }

        /// <summary>
        /// Cleans up unmanaged GPU resources when the renderer is destroyed.
        /// </summary>
        public void Dispose()
        {
            cachedPlaneMeshRef?.Dispose();
        }
    }
}