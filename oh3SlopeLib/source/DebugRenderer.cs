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
        private MeshRef cachedPlaneMeshRef;

        /// <summary>
        /// Controls whether the debug visuals are actively drawn.
        /// </summary>
        public bool IsActive { get; set; } = false;

        public SlopeDebugRenderer(ICoreClientAPI capi, string hexColor)
        {
            this.capi = capi;
            BuildGlowingPlaneMesh(1.5f, hexColor);
        }

        /// <summary>
        /// 1. Initialization (Build the Mesh Once)
        /// </summary>
        private void BuildGlowingPlaneMesh(float size, string hexColor)
        {
            // Capacity: 5 vertices, 10 indices. Include UVs and Normals for Standard shader compatibility.
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
            mesh.Uv = new float[10];
            mesh.Normals = new int[5];
            mesh.VerticesCount = 5;

            // Indices: 8 to draw the perimeter of the quad, 2 to draw the line from center to tip
            mesh.Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                0, 4, 2, 4
            };
            mesh.IndicesCount = 10;

            byte r = 0, g = 255, b = 255, a = 255;
            try
            {
                string hex = hexColor?.TrimStart('#') ?? "00FFFF";
                if (hex.Length >= 6)
                {
                    r = byte.Parse(hex.Substring(0, 2), System.Globalization.NumberStyles.HexNumber);
                    g = byte.Parse(hex.Substring(2, 2), System.Globalization.NumberStyles.HexNumber);
                    b = byte.Parse(hex.Substring(4, 2), System.Globalization.NumberStyles.HexNumber);
                    if (hex.Length == 8) a = byte.Parse(hex.Substring(6, 2), System.Globalization.NumberStyles.HexNumber);
                }
            }
            catch { /* Keep default cyan on parse failure */ }

            // Colors: 4 bytes per vertex (RGBA)
            mesh.Rgba = new byte[] {
                r, g, b, a, // Base 1
                r, g, b, a, // Base 2
                r, g, b, a, // Base 3
                r, g, b, a, // Base 4
                255, 255, 255, 255  // Tip (White)
            };

            // Glow: Iterate through the mesh.Flags array and set each to max brightness
            mesh.Flags = new int[] {
                128 << 12, 128 << 12, 128 << 12, 128 << 12, 128 << 12
            };

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

            // Use the strongly typed StandardShader to avoid KeyNotFoundExceptions
            IStandardShaderProgram prog = capi.Render.StandardShader;
            prog.Use();
            prog.RgbaTint = new Vec4f(1, 1, 1, 1);
            prog.ExtraGlow = 255;
            prog.ProjectionMatrix = capi.Render.CurrentProjectionMatrix;
            prog.ViewMatrix = capi.Render.CameraMatrixOriginf;

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
                Vec3d camPos = capi.World.Player.Entity.CameraPos;
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