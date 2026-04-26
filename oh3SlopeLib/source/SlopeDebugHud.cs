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
    /// <remarks>
    /// Designed to execute continuously during the client render loop. Utilizes a persistent 
    /// <see cref="StringBuilder"/> to completely eliminate string allocation Garbage Collection (GC) spikes.
    /// </remarks>
    public class SlopeDebugHud : GuiDialog
    {
        /// <summary>
        /// The physics behavior actively tracked by the local player.
        /// </summary>
        private EntityBehaviorSlopeAware behavior;

        /// <summary>
        /// Reusable string builder to prevent massive memory allocations during continuous string concatenation.
        /// </summary>
        private StringBuilder sb = new StringBuilder();

        /// <summary>
        /// Accumulates delta time to throttle text rendering updates.
        /// </summary>
        private float updateTimer = 0f;

        /// <summary>
        /// Tracks consecutive exceptions to gracefully throttle error logging without crippling the game thread.
        /// </summary>
        private int consecutiveErrors = 0;

        /// <summary>
        /// Defines the keybind required to open this dialog. Returning null prevents this dialog 
        /// from capturing standard input keys (like ESC or E) when active.
        /// </summary>
        public override string ToggleKeyCombinationCode => null;

        /// <summary>
        /// Categorizes the dialog within the engine. Setting this to HUD ensures it renders passively 
        /// and does not steal mouse or camera control from the player.
        /// </summary>
        public override EnumDialogType DialogType => EnumDialogType.HUD;

        /// <summary>
        /// Determines if the dialog can receive user focus. Forced to false to ensure passive visualization.
        /// </summary>
        public override bool Focusable => false;

        /// <summary>
        /// Initializes a new instance of the <see cref="SlopeDebugHud"/> class.
        /// </summary>
        /// <param name="capi">The core client API used for GUI construction and rendering.</param>
        /// <param name="behavior">The initial slope behavior to track.</param>
        public SlopeDebugHud(ICoreClientAPI capi, EntityBehaviorSlopeAware behavior) : base(capi)
        {
            this.behavior = behavior;
            SetupDialog();
        }

        /// <summary>
        /// Constructs the geometric bounds and visual elements of the dialog.
        /// </summary>
        private void SetupDialog()
        {
            // Position the logger cleanly in the top left, clear of central crosshairs.
            // Width and Height are hardcoded to comfortably fit 4 surface planes per sphere.
            ElementBounds dialogBounds = ElementBounds.Fixed(EnumDialogArea.LeftTop, 10, 10, 550, 150);

            // Explicitly parent the bounds so ElementBounds.Fill automatically constrains itself to the dialog's dimensions
            ElementBounds bgBounds = ElementBounds.Fill;
            ElementBounds textBounds = ElementBounds.Fill.WithFixedPadding(10);
            dialogBounds.WithChildren(bgBounds, textBounds);

            // Compile the GUI composition and mount it to the active interface
            SingleComposer = capi.Gui.CreateCompo("slopedebuglogger", dialogBounds)
                .AddDialogBG(bgBounds, false)
                .AddDynamicText("", CairoFont.WhiteDetailText(), textBounds, "loggerText")
                .Compose();
        }

        /// <summary>
        /// Updates the HUD text every render frame to display the live physics behavior outputs.
        /// </summary>
        /// <param name="deltaTime">The time elapsed since the last GUI render tick.</param>
        public override void OnRenderGUI(float deltaTime)
        {
            base.OnRenderGUI(deltaTime);

            // OPTIMIZATION: Throttle UI updates to ~20 times per second. 
            // Text rendering is expensive; we don't need to rebuild strings at 144hz.
            updateTimer += deltaTime;
            if (updateTimer < 0.05f) return;
            updateTimer = 0f;

            // FIX #1: Dynamically fetch the behavior on the active frame. This ensures we never hold a 
            // stale or invalid reference if the behavior is removed and re-added by the debug toggle command!
            behavior = capi?.World?.Player?.Entity?.GetBehavior<EntityBehaviorSlopeAware>();

            // Null-guards protect against partially initialized entities during chunk loading edges
            if (behavior?.SurfaceDataList == null) return;

            try
            {
                // OPTIMIZATION: Clear the builder and format everything in-place to ensure zero string orphan allocations.
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

                        // Treat distances near our initial arbitrary 9999.0 boundary as unresolved/empty slots
                        if (surf == null || surf.DistanceToSurface > 9990.0)
                        {
                            sb.AppendFormat("  ► Surf {0}: [No Surface]\n", i);
                            continue;
                        }

                        Block physicalBlock = behavior.GetPhysicalSurfaceBlock(capi.World.BlockAccessor, surf.SurfaceNormal, surf.SurfacePoint.X, surf.SurfacePoint.Y, surf.SurfacePoint.Z);

                        // Safe-navigation operators (?.) prevent exceptions if a block doesn't have a valid Code during asynchronous chunk loading
                        string blockName = physicalBlock?.Code?.Path ?? "None";
                        string materialName = physicalBlock != null ? physicalBlock.BlockMaterial.ToString() : "None";

                        sb.AppendFormat("  ► Surf {0}: Norm: [{1:F2}, {2:F2}, {3:F2}] | Dist: {4:F2}\n", i, surf.SurfaceNormal.X, surf.SurfaceNormal.Y, surf.SurfaceNormal.Z, surf.DistanceToSurface);
                        sb.AppendFormat("    Point: [{0:F2}, {1:F2}, {2:F2}] | Mat: {3} ({4})\n", surf.SurfacePoint.X, surf.SurfacePoint.Y, surf.SurfacePoint.Z, materialName, blockName);
                    }
                    sb.AppendLine(); // Aesthetic spacing between evaluated spheres
                }

                // Push the unified string block to the rendering component
                SingleComposer.GetDynamicText("loggerText").SetNewText(sb.ToString());

                // If the entire frame executed perfectly, reset the error throttle
                consecutiveErrors = 0;
            }
            catch (Exception e)
            {
                // Throttle log output to prevent severe I/O lag from continuous log spam when geometry queries fail
                if (consecutiveErrors < 5)
                {
                    capi.Logger.VerboseDebug($"[SlopeLib] Suppressed error in Debug HUD: {e.Message}");
                    consecutiveErrors++;
                }
            }
        }

        /// <summary>
        /// Cleans up unmanaged resources and internal states when the dialog is closed or the mod unloads.
        /// </summary>
        public override void Dispose()
        {
            base.Dispose();
            // Explicitly release Cairo font textures and UI composition buffers to prevent memory leaks across play sessions
            SingleComposer?.Dispose();
            SingleComposer = null;
        }
    }
}