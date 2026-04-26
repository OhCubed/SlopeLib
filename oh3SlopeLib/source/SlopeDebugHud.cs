using System;
using System.Text;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// A lightweight HUD overlay for visualizing slope physics data in real-time.
    /// </summary>
    public class SlopeDebugHud : GuiDialog
    {
        private EntityBehaviorSlopeAware behavior;

        // Reusable string builder to prevent massive memory allocations during string concatenation
        private StringBuilder sb = new StringBuilder();
        private float updateTimer = 0f;

        // Null prevents this dialog from capturing standard input keys like ESC or E
        public override string ToggleKeyCombinationCode => null;

        // Setting the dialog type to HUD prevents it from stealing mouse/camera control
        public override EnumDialogType DialogType => EnumDialogType.HUD;
        public override bool Focusable => false;

        public SlopeDebugHud(ICoreClientAPI capi, EntityBehaviorSlopeAware behavior) : base(capi)
        {
            this.behavior = behavior;
            SetupDialog();
        }

        private void SetupDialog()
        {
            // Position the logger cleanly in the top left, clear of central crosshairs.
            ElementBounds dialogBounds = ElementBounds.Fixed(EnumDialogArea.LeftTop, 10, 10, 500, 50);

            // Explicitly parent the bounds so ElementBounds.Fill knows what to constrain to
            ElementBounds bgBounds = ElementBounds.Fill;
            ElementBounds textBounds = ElementBounds.Fill.WithFixedPadding(10);
            dialogBounds.WithChildren(bgBounds, textBounds);

            SingleComposer = capi.Gui.CreateCompo("slopedebuglogger", dialogBounds)
                .AddDialogBG(bgBounds, false)
                .AddDynamicText("", CairoFont.WhiteDetailText(), textBounds, "loggerText")
                .Compose();
        }

        /// <summary>
        /// Updates the HUD text every render frame to display the live behavior outputs.
        /// </summary>
        public override void OnRenderGUI(float deltaTime)
        {
            base.OnRenderGUI(deltaTime);

            // Throttle UI updates to ~20 times per second to prevent massive string allocation GC spikes
            updateTimer += deltaTime;
            if (updateTimer < 0.05f) return;
            updateTimer = 0f;

            // FIX #1: Dynamically fetch the behavior on the active frame. This ensures we never hold a 
            // stale reference if the behavior is removed and re-added by the debug toggle command!
            behavior = capi?.World?.Player?.Entity?.GetBehavior<EntityBehaviorSlopeAware>();

            if (behavior == null) return;

            try
            {
                Vec3d normal = behavior.SurfaceNormal;
                Vec3d point = behavior.SurfacePoint;

                // Emulate the raycast to find the actual physical block underneath the projected point
                Block physicalBlock = behavior.GetPhysicalSurfaceBlock(capi.World.BlockAccessor, point.X, point.Y, point.Z);

                // Safe-navigation operators prevent exceptions if a block doesn't have a valid Code during chunk loading
                string blockName = physicalBlock?.Code?.Path ?? "None";
                string materialName = physicalBlock != null ? physicalBlock.BlockMaterial.ToString() : "None";

                // Clear the builder and format everything in-place to ensure zero string orphan allocations
                sb.Clear();

                if (normal != null && point != null)
                {
                    sb.AppendFormat("Normal: [{0:F2}, {1:F2}, {2:F2}]  |  Point: [{3:F2}, {4:F2}, {5:F2}]\n", normal.X, normal.Y, normal.Z, point.X, point.Y, point.Z);
                }
                else
                {
                    sb.AppendLine("Normal: [None]  |  Point: [None]");
                }

                sb.AppendFormat("Sphere Dia: {0:F2}  |  Y-Offset: {1:F2}  |  Dist To Plane: {2:F2}\n", behavior.CollisionSphereSize, behavior.CollisionSphereYOffset, behavior.DistanceToSurface);
                sb.AppendFormat("Physical Block: {0}  |  Material: {1}", blockName, materialName);

                SingleComposer.GetDynamicText("loggerText").SetNewText(sb.ToString());
            }
            catch (Exception e)
            {
                // Gracefully suppress exceptions during render passes so the client game doesn't hard-crash
                capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error in Debug HUD: {e.Message}");
            }
        }

        public override void Dispose()
        {
            base.Dispose();
            // Explicitly release Cairo font textures/buffers to prevent memory leaks across play sessions
            SingleComposer?.Dispose();
        }
    }
}