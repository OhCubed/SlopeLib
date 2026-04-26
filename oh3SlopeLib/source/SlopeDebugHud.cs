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

        // --- Error state tracking to gracefully degrade without spamming logs ---
        private int consecutiveErrors = 0;

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
            ElementBounds dialogBounds = ElementBounds.Fixed(EnumDialogArea.LeftTop, 10, 10, 550, 450);

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

            // Null-guards protect against partially initialized entities during chunk loading edges
            if (behavior?.SurfaceDataList == null) return;

            try
            {
                // Clear the builder and format everything in-place to ensure zero string orphan allocations
                sb.Clear();
                sb.AppendFormat("Tracked Spheres: {0}\n\n", behavior.SurfaceDataList.Length);

                for (int s = 0; s < behavior.SurfaceDataList.Length; s++)
                {
                    var sd = behavior.SurfaceDataList[s];
                    sb.AppendFormat("S{0} [Dia: {1:F1} | X/Y/Z Offsets: {2:F1}, {3:F1}, {4:F1}]\n", s, sd.CollisionSphereSize, sd.CollisionSphereXOffset, sd.CollisionSphereYOffset, sd.CollisionSphereZOffset);

                    if (sd.Surfaces == null) continue;

                    for (int i = 0; i < sd.Surfaces.Length; i++)
                    {
                        var surf = sd.Surfaces[i];
                        if (surf == null || surf.DistanceToSurface > 9990.0)
                        {
                            sb.AppendFormat("  ► Surf {0}: [No Surface]\n", i);
                            continue;
                        }

                        Block physicalBlock = behavior.GetPhysicalSurfaceBlock(capi.World.BlockAccessor, surf.SurfaceNormal, surf.SurfacePoint.X, surf.SurfacePoint.Y, surf.SurfacePoint.Z);

                        // Safe-navigation operators prevent exceptions if a block doesn't have a valid Code during chunk loading
                        string blockName = physicalBlock?.Code?.Path ?? "None";
                        string materialName = physicalBlock != null ? physicalBlock.BlockMaterial.ToString() : "None";

                        sb.AppendFormat("  ► Surf {0}: Norm: [{1:F2}, {2:F2}, {3:F2}] | Dist: {4:F2}\n", i, surf.SurfaceNormal.X, surf.SurfaceNormal.Y, surf.SurfaceNormal.Z, surf.DistanceToSurface);
                        sb.AppendFormat("    Point: [{0:F2}, {1:F2}, {2:F2}] | Mat: {3} ({4})\n", surf.SurfacePoint.X, surf.SurfacePoint.Y, surf.SurfacePoint.Z, materialName, blockName);
                    }
                    sb.AppendLine(); // Spacing between spheres
                }

                SingleComposer.GetDynamicText("loggerText").SetNewText(sb.ToString());

                // If the entire frame rendered perfectly, clear the error counter
                consecutiveErrors = 0;
            }
            catch (Exception e)
            {
                // Throttle log output to prevent severe I/O lag from continuous log spam
                if (consecutiveErrors < 5)
                {
                    capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error in Debug HUD: {e.Message}");
                    consecutiveErrors++;
                }
            }
        }

        public override void Dispose()
        {
            base.Dispose();
            // Explicitly release Cairo font textures/buffers to prevent memory leaks across play sessions
            SingleComposer?.Dispose();
            SingleComposer = null;
        }
    }
}