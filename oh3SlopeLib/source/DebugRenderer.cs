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
    /// <remarks>
    /// Designed with a strict zero-allocation render loop. Matrices and vectors are pre-allocated 
    /// and mutated in-place to prevent Garbage Collection spikes during the opaque render pass.
    /// </remarks>
    public class SlopeDebugRenderer : IRenderer
    {
        // --- Core Dependencies ---
        private ICoreClientAPI capi;
        private oh3SlopeLibModSystem modSys;

        /// <summary>
        /// A handle to the unmanaged GPU memory containing the custom debug plane geometry.
        /// Must be explicitly disposed of to prevent VRAM leaks.
        /// </summary>
        private MeshRef cachedPlaneMeshRef;

        // --- Pre-allocated memory buffers to prevent GC spikes in the render loop ---

        /// <summary>A reusable 4x4 matrix for transforming the debug plane into world space.</summary>
        private float[] modelMatrix = Mat4f.Create();

        /// <summary>A reusable 3D vector for computing the axis of rotation to align the plane.</summary>
        private float[] rotationAxis = new float[3];

        /// <summary>An overriding light vector to ensure the debug planes render at maximum brightness.</summary>
        private Vec4f overrideLight = new Vec4f(1f, 1f, 1f, 1f);

        /// <summary>An overriding ambient light vector to prevent the planes from being darkened by shadows.</summary>
        private Vec3f overrideAmbient = new Vec3f(1f, 1f, 1f);

        // --- Error state tracking to gracefully degrade without spamming logs ---
        private int consecutiveErrors = 0;

        /// <summary>
        /// Gets or sets a value indicating whether the debug visuals should be rendered this frame.
        /// </summary>
        public bool IsActive { get; set; } = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="SlopeDebugRenderer"/> class.
        /// </summary>
        /// <param name="capi">The core client API utilized for rendering and entity fetching.</param>
        /// <param name="modSys">The parent mod system utilized for configuration fetching.</param>
        public SlopeDebugRenderer(ICoreClientAPI capi, oh3SlopeLibModSystem modSys)
        {
            this.capi = capi;
            this.modSys = modSys;
            BuildGlowingPlaneMesh(1.5f);
        }

        /// <summary>
        /// Constructs the geometric mesh data for the debug plane and uploads it to the GPU.
        /// </summary>
        /// <param name="size">The length of the directional normal indicator line.</param>
        private void BuildGlowingPlaneMesh(float size)
        {
            // Ensure any existing mesh is disposed of to prevent VRAM memory leaks during hot-reloads
            cachedPlaneMeshRef?.Dispose();

            // Capacity: 6 vertices, 10 indices. The standard shader requires UVs, Normals, and Flags.
            MeshData mesh = new MeshData(6, 10, true, true, true, true);
            mesh.SetMode(EnumDrawMode.Lines);

            // Define a flat 1x1 quad centered at the origin, accompanied by a directional line pointing upwards.
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

            // Specify vertex indices: 8 indices for the perimeter quad, 2 indices for the directional normal line.
            mesh.Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                5, 4
            };
            mesh.IndicesCount = 10;

            // Initialize default color (white), UV coordinates, and vertex flags. 
            // The shader will apply the dynamic configuration tint over this base white.
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

            // Upload the constructed mesh geometry to the GPU and cache the handle.
            cachedPlaneMeshRef = capi.Render.UploadMesh(mesh);
        }

        /// <summary>
        /// Defines the execution order relative to other renderers. 0.5 designates standard opaque priority.
        /// </summary>
        public double RenderOrder => 0.5;

        /// <summary>
        /// Defines the maximum distance in blocks from the camera at which this renderer will execute.
        /// </summary>
        public int RenderRange => 24;

        /// <summary>
        /// Associates this renderer with the standard opaque rendering pipeline stage.
        /// </summary>
        public EnumRenderStage Stage => EnumRenderStage.Opaque;

        /// <summary>
        /// Executed by the engine every frame during the designated render stage.
        /// Iterates over all active entities and renders planes representing their tracked surface geometry.
        /// </summary>
        /// <param name="deltaTime">The time elapsed since the last render frame.</param>
        /// <param name="stage">The current render stage being processed.</param>
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

            // Wrap rendering logic in a try-catch-finally block to prevent localized math exceptions 
            // from cascading, while guaranteeing the shader always unbinds safely.
            try
            {
                // OPTIMIZATION: Iterate the ConcurrentDictionary via KeyValuePair. Calling .Values 
                // dynamically allocates a new array and ReadOnlyCollection, which causes severe GC stutters.
                foreach (var kvp in capi.World.LoadedEntities)
                {
                    var entity = kvp.Value;

                    // Exclude dead or invalid entities
                    if (entity == null || !entity.Alive) continue;

                    var tracker = entity.GetBehavior<EntityBehaviorSlopeAware>();

                    // Null-guards protect against partially initialized entities during chunk loading edges
                    if (tracker?.SurfaceDataList == null) continue;

                    // Iterate through every active sampling sphere attached to the entity
                    for (int s = 0; s < tracker.SurfaceDataList.Length; s++)
                    {
                        var sd = tracker.SurfaceDataList[s];
                        if (sd?.Surfaces == null) continue;

                        // Render up to 4 tracked distinct surfaces for each sphere
                        for (int i = 0; i < sd.Surfaces.Length; i++)
                        {
                            var surf = sd.Surfaces[i];

                            // Skip rendering uninitialized, null, or empty surface slots (indicated by distance > 9990)
                            if (surf == null || surf.DistanceToSurface > 9990.0) continue;

                            Vec3d normal = surf.SurfaceNormal;
                            Vec3d pos = surf.SurfacePoint;

                            if (normal == null || pos == null) continue;

                            // OPTIMIZATION: Cast components explicitly to avoid creating temporary Vec3f objects.
                            float nX = (float)normal.X;
                            float nY = (float)normal.Y;
                            float nZ = (float)normal.Z;

                            // Shrink lower-priority trailing planes slightly so they are visually distinguishable
                            // Primary = 1.0 scale, Pass 1 = 0.85 scale, Pass 2 = 0.70 scale, etc.
                            float visualScale = 1f - (i * 0.15f);

                            // Initialize the pre-allocated model matrix buffer with a scaled pristine identity state.
                            // This manual assignment bypasses the method invocation overhead of Mat4f.Identity().
                            modelMatrix[0] = visualScale; modelMatrix[1] = 0f; modelMatrix[2] = 0f; modelMatrix[3] = 0f;
                            modelMatrix[4] = 0f; modelMatrix[5] = visualScale; modelMatrix[6] = 0f; modelMatrix[7] = 0f;
                            modelMatrix[8] = 0f; modelMatrix[9] = 0f; modelMatrix[10] = visualScale; modelMatrix[11] = 0f;

                            // Apply translation relative to the camera's current viewport.
                            modelMatrix[12] = (float)(pos.X - camX);
                            modelMatrix[13] = (float)(pos.Y - camY);
                            modelMatrix[14] = (float)(pos.Z - camZ);
                            modelMatrix[15] = 1f;

                            // Determine the axis of rotation using an optimized, inline cross-product.
                            // This calculates the rotation necessary to align the UP vector (0, 1, 0) with the surface normal.
                            float lenSq = nZ * nZ + nX * nX;
                            float len = lenSq > 0.000001f ? (float)Math.Sqrt(lenSq) : 0f;

                            if (len > 0.0001f)
                            {
                                rotationAxis[0] = nZ / len;
                                rotationAxis[1] = 0;
                                rotationAxis[2] = -nX / len;

                                // Inline clamp to mathematically guarantee the dot product rests between -1 and 1.
                                // This entirely removes the risk of Math.Acos throwing a NaN exception.
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
                }

                // If the entire frame rendered perfectly, clear the error counter
                consecutiveErrors = 0;
            }
            catch (Exception e)
            {
                // Throttle log output to prevent severe I/O lag from continuous log spam
                if (consecutiveErrors < 5)
                {
                    capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error during debug render: {e.Message}");
                    consecutiveErrors++;
                }
            }
            finally
            {
                // Guarantee the shader unbinds even if a severe thread exception occurs,
                // preventing graphical corruption in other renderer passes.
                prog.Stop();
            }
        }

        /// <summary>
        /// Cleans up unmanaged GPU resources when the renderer is destroyed or the client disconnects.
        /// </summary>
        public void Dispose()
        {
            // Release the unmanaged OpenGL Mesh memory
            cachedPlaneMeshRef?.Dispose();
            cachedPlaneMeshRef = null;
        }
    }
}