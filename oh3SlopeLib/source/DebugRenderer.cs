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
            // Capacity: 5 vertices, 10 indices. Standard shader requires UVs, Normals, and Flags.
            MeshData mesh = new MeshData(5, 10, true, true, true, true);
            mesh.SetMode(EnumDrawMode.Lines);

            // Vertices: 4 form a flat quad on the X/Z axis at Y=0, 1 sits at (0, size, 0)
            mesh.xyz = new float[] {
                -0.5f, 0, -0.5f,
                 0.5f, 0, -0.5f,
                 0.5f, 0,  0.5f,
                -0.5f, 0,  0.5f,
                 0, size, 0
            };
            mesh.VerticesCount = 5;

            mesh.Uv = new float[10];
            mesh.Normals = new int[5];
            mesh.Flags = new int[5];

            // Indices: 8 to draw the perimeter of the quad, 2 to draw the line from center to tip
            mesh.Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                0, 4, 2, 4
            };
            mesh.IndicesCount = 10;

            // Standard shader expects the mesh to be white, and tinted via RgbaTint later
            mesh.Rgba = new byte[20];
            for (int i = 0; i < 5; i++)
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
            // Abort early if the debug view is toggled off
            if (!IsActive || cachedPlaneMeshRef == null) return;

            // Use StandardShader matching the working example
            IStandardShaderProgram prog = capi.Render.StandardShader;
            if (prog == null) return;

            prog.Use();
            prog.DontWarpVertices = 1;

            // Mimic the exact lighting and coloring logic that worked in the previous mod
            prog.RgbaTint = modSys.DebugColorVec;
            prog.RgbaLightIn = new Vec4f(1f, 1f, 1f, 1f);
            prog.RgbaAmbientIn = new Vec3f(1f, 1f, 1f);
            prog.ExtraGlow = 255;
            prog.ProjectionMatrix = capi.Render.CurrentProjectionMatrix;
            prog.ViewMatrix = capi.Render.CameraMatrixOriginf;

            Vec3d camPos = capi.World.Player.Entity.CameraPos;

            foreach (var entity in capi.World.LoadedEntities.Values)
            {
                var tracker = entity.GetBehavior<EntityBehaviorSlopeAware>();
                if (tracker == null) continue;

                // 1. Get data from our tracker behavior
                Vec3d normal = tracker.SurfaceNormal;
                Vec3d pos = entity.Pos.XYZ;

                // 2. Calculate rotation from standard "Up" to our normal (Using Axis-Angle)
                Vec3f up = new Vec3f(0, 1, 0);
                Vec3f n = normal.ToVec3f();

                float[] axis = new float[] {
                    up.Y * n.Z - up.Z * n.Y,
                    up.Z * n.X - up.X * n.Z,
                    up.X * n.Y - up.Y * n.X
                };
                float len = (float)Math.Sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
                float dot = up.X * n.X + up.Y * n.Y + up.Z * n.Z;
                float angle = (float)Math.Acos(GameMath.Clamp(dot, -1f, 1f));

                // 3. Render Setup
                float[] modelMatrix = Mat4f.Create();

                // 4. Position relative to camera
                Mat4f.Translate(modelMatrix, modelMatrix, (float)(pos.X - camPos.X), (float)(pos.Y - camPos.Y), (float)(pos.Z - camPos.Z));

                // Apply the calculated rotation to our custom ModelMatrix
                if (len > 0.0001f)
                {
                    axis[0] /= len;
                    axis[1] /= len;
                    axis[2] /= len;
                    Mat4f.Rotate(modelMatrix, modelMatrix, angle, axis);
                }
                else if (dot < 0)
                {
                    // Fallback for looking straight down
                    Mat4f.Rotate(modelMatrix, modelMatrix, (float)Math.PI, new float[] { 1, 0, 0 });
                }

                // 5. Draw and Cleanup
                prog.ModelMatrix = modelMatrix;

                capi.Render.RenderMesh(cachedPlaneMeshRef);
            }

            prog.Stop();
        }

        public void Dispose()
        {
            cachedPlaneMeshRef?.Dispose();
        }
    }
}