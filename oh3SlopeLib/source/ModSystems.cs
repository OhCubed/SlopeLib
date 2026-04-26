using Vintagestory.API.Common;
using Vintagestory.API.Server;
using Vintagestory.API.Client;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// The primary entry point and orchestration system for the SlopeLib mod.
    /// Handles cross-side behavior registration, configuration management, 
    /// and initializes the client-side debug rendering pipeline.
    /// </summary>
    public sealed class oh3SlopeLibModSystem : ModSystem
    {
        // --- Core Client References ---
        private ICoreClientAPI capi;
        private SlopeDebugRenderer debugRenderer;
        private SlopeDebugHud debugHud;

        /// <summary>
        /// Retrieves the active server-side configuration parameters (e.g., global physics toggles).
        /// </summary>
        public SlopeLibServerConfig ServerConfig { get; private set; }

        /// <summary>
        /// Retrieves the active client-side configuration parameters (e.g., debug UI colors).
        /// </summary>
        public SlopeLibClientConfig ClientConfig { get; private set; }

        /// <summary>
        /// The parsed, normalized color vector utilized by the debug shader. 
        /// Represented as RGBA floats between 0.0 and 1.0.
        /// </summary>
        public Vec4f DebugColorVec { get; private set; } = new Vec4f(0f, 1f, 1f, 1f);

        /// <summary>
        /// Executed during the initial startup phase on both the client and the server.
        /// Primarily used to register shared engine components like Entity Behaviors.
        /// </summary>
        /// <param name="api">The unified core API provided by the engine.</param>
        public override void Start(ICoreAPI api)
        {
            base.Start(api);

            // Register the custom slope-aware physics behavior with the engine's entity system
            // so that it can be applied to entities via their JSON definitions.
            api.RegisterEntityBehaviorClass("slopeaware", typeof(EntityBehaviorSlopeAware));
        }

        /// <summary>
        /// Executed exclusively during the server-side startup phase.
        /// </summary>
        /// <param name="api">The server-specific API providing access to server configs and world generation.</param>
        public override void StartServerSide(ICoreServerAPI api)
        {
            base.StartServerSide(api);

            try
            {
                // Attempt to deserialize the server configuration. If the file does not exist,
                // fall back to a fresh instance and immediately serialize it to disk to create the template.
                ServerConfig = api.LoadModConfig<SlopeLibServerConfig>("oh3SlopeLibServerConfig.json") ?? new SlopeLibServerConfig();
                api.StoreModConfig(ServerConfig, "oh3SlopeLibServerConfig.json");
            }
            catch (System.Exception e)
            {
                ServerConfig = new SlopeLibServerConfig();
                api.Logger.Error("Failed to load server config! Falling back to default settings. Error: {0}", e);
            }
        }

        /// <summary>
        /// Executed exclusively during the client-side startup phase.
        /// Responsible for mounting UI components, rendering hooks, and client commands.
        /// </summary>
        /// <param name="api">The client-specific API providing access to rendering, input, and GUI orchestration.</param>
        public override void StartClientSide(ICoreClientAPI api)
        {
            this.capi = api;
            base.StartClientSide(api);

            try
            {
                // Attempt to deserialize the client configuration, generating a default file if necessary.
                ClientConfig = api.LoadModConfig<SlopeLibClientConfig>("oh3SlopeLibClientConfig.json") ?? new SlopeLibClientConfig();
                api.StoreModConfig(ClientConfig, "oh3SlopeLibClientConfig.json");
            }
            catch (System.Exception e)
            {
                ClientConfig = new SlopeLibClientConfig();
                api.Logger.Error("Failed to load client config! Falling back to default settings. Error: {0}", e);
            }

            ParseDebugColor();

            // Initialize and register the debug rendering pipeline into the engine's opaque geometry pass.
            // The ModSystem instance is passed down to allow the renderer dynamic access to the parsed color vector.
            debugRenderer = new SlopeDebugRenderer(api, this);
            api.Event.RegisterRenderer(debugRenderer, EnumRenderStage.Opaque, "slopedebug");

            // Construct and register the interactive client-side developer command tree.
            api.ChatCommands.GetOrCreate("slopelib")
                .WithDescription("SlopeLib client commands")
                .BeginSubCommand("debug")
                    .WithDescription("Toggles the slope engine debug visualizer")
                    .WithArgs(
                        api.ChatCommands.Parsers.OptionalDouble("diameter", -9999.0),
                        api.ChatCommands.Parsers.OptionalDouble("yoffset", -9999.0)
                    )
                    .HandleWith(OnToggleDebug)
                .EndSubCommand();
        }

        /// <summary>
        /// Translates the configured HTML hex color string (e.g., "#FF00FF") into a 
        /// normalized OpenGL Vec4f suitable for immediate shader consumption.
        /// </summary>
        /// <remarks>Gracefully falls back to a bright cyan (0, 1, 1, 1) if the string is malformed.</remarks>
        private void ParseDebugColor()
        {
            string hex = ClientConfig?.DebugColor?.TrimStart('#') ?? "00FFFF";
            if (hex.Length >= 6)
            {
                try
                {
                    float r = int.Parse(hex.Substring(0, 2), System.Globalization.NumberStyles.HexNumber) / 255f;
                    float g = int.Parse(hex.Substring(2, 2), System.Globalization.NumberStyles.HexNumber) / 255f;
                    float b = int.Parse(hex.Substring(4, 2), System.Globalization.NumberStyles.HexNumber) / 255f;
                    float a = hex.Length == 8 ? int.Parse(hex.Substring(6, 2), System.Globalization.NumberStyles.HexNumber) / 255f : 1f;
                    DebugColorVec = new Vec4f(r, g, b, a);
                }
                catch
                {
                    // Retain the default cyan fallback upon parsing failure to ensure visualizer stability.
                }
            }
        }

        /// <summary>
        /// Handles the execution of the '/slopelib debug' chat command.
        /// Dynamically attaches or detaches the visualizer and evaluation behavior to the local player entity.
        /// </summary>
        /// <param name="args">The parsed arguments provided by the user in the chat console.</param>
        /// <returns>A command result dictating success or failure, outputted to the player's chat log.</returns>
        private TextCommandResult OnToggleDebug(TextCommandCallingArgs args)
        {
            if (debugRenderer == null) return TextCommandResult.Error("Debug renderer is not initialized.");

            // Safely cast the parsed arguments. The predefined -9999.0 values serve as sentinel 
            // indicators to determine if the user explicitly provided dimensional overrides.
            double diameter = args[0] as double? ?? -9999.0;
            double yoffset = args[1] as double? ?? -9999.0;
            bool hasArgs = diameter > -9990.0;

            // Enforce activation if explicit arguments are provided; otherwise, toggle the current active state.
            if (hasArgs)
            {
                debugRenderer.IsActive = true;
            }
            else
            {
                debugRenderer.IsActive = !debugRenderer.IsActive;
            }

            var playerEntity = capi.World.Player.Entity;

            if (debugRenderer.IsActive)
            {
                // Ensure the local player entity possesses the behavior necessary for geometry testing.
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior == null)
                {
                    behavior = new EntityBehaviorSlopeAware(playerEntity);
                    playerEntity.AddBehavior(behavior);
                }

                // Immediately apply explicit configurations to the active behavior instance via the zero-allocation updater.
                if (hasArgs)
                {
                    if (behavior.SurfaceDataList != null && behavior.SurfaceDataList.Length > 0)
                    {
                        behavior.UpdateSphereConfig(
                            0,
                            diameter,
                            behavior.SurfaceDataList[0].CollisionSphereXOffset,
                            yoffset > -9990.0 ? yoffset : (diameter / 2.0),
                            behavior.SurfaceDataList[0].CollisionSphereZOffset
                        );
                    }
                }

                // Mount and display the real-time HUD logger.
                if (debugHud == null) debugHud = new SlopeDebugHud(capi, behavior);
                debugHud.TryOpen();

                if (hasArgs)
                {
                    return TextCommandResult.Success($"SlopeLib debug view is now ON (Diameter: {behavior.SurfaceDataList[0].CollisionSphereSize}, Y-Offset: {behavior.SurfaceDataList[0].CollisionSphereYOffset}).");
                }
            }
            else
            {
                // Cleanly detach the behavior to entirely halt math processing overhead when debug mode is disabled.
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior != null)
                {
                    playerEntity.RemoveBehavior(behavior);
                }

                // Unmount and hide the HUD logger.
                debugHud?.TryClose();
            }

            return TextCommandResult.Success($"SlopeLib debug view is now {(debugRenderer.IsActive ? "ON" : "OFF")}.");
        }

        /// <summary>
        /// Executed when the mod is unloaded or the client disconnects.
        /// Responsible for freeing unmanaged memory and wiping static caches.
        /// </summary>
        public override void Dispose()
        {
            base.Dispose();

            // Destroy the HUD dialog to free Cairo font textures and UI buffers.
            debugHud?.Dispose();
            debugHud = null;

            // CRITICAL OPTIMIZATION: We must dispose of our OpenGL MeshRef when the mod unloads 
            // to prevent VRAM memory leaks every time the player disconnects and reconnects.
            debugRenderer?.Dispose();
            debugRenderer = null;

            // Flush the compiled expression tree delegates to prevent static types from leaking across world reloads.
            MaterialUtility.ClearCache();
        }
    }
}