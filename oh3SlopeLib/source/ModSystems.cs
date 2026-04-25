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
        private ICoreClientAPI capi;
        private SlopeDebugRenderer debugRenderer;

        /// <summary>
        /// Retrieves the active server-side configuration parameters.
        /// </summary>
        public SlopeLibServerConfig ServerConfig { get; private set; }

        /// <summary>
        /// Retrieves the active client-side configuration parameters.
        /// </summary>
        public SlopeLibClientConfig ClientConfig { get; private set; }

        /// <summary>
        /// The parsed, normalized color vector utilized by the debug shader.
        /// </summary>
        public Vec4f DebugColorVec { get; private set; } = new Vec4f(0f, 1f, 1f, 1f);

        /// <summary>
        /// Executed during the initial startup phase on both the client and the server.
        /// </summary>
        public override void Start(ICoreAPI api)
        {
            base.Start(api);

            // Register the custom slope-aware physics behavior with the engine's entity system.
            api.RegisterEntityBehaviorClass("slopeaware", typeof(EntityBehaviorSlopeAware));
        }

        /// <summary>
        /// Executed during the server-side startup phase.
        /// </summary>
        public override void StartServerSide(ICoreServerAPI api)
        {
            base.StartServerSide(api);

            try
            {
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
        /// Executed during the client-side startup phase.
        /// </summary>
        public override void StartClientSide(ICoreClientAPI api)
        {
            this.capi = api;
            base.StartClientSide(api);

            try
            {
                ClientConfig = api.LoadModConfig<SlopeLibClientConfig>("oh3SlopeLibClientConfig.json") ?? new SlopeLibClientConfig();
                api.StoreModConfig(ClientConfig, "oh3SlopeLibClientConfig.json");
            }
            catch (System.Exception e)
            {
                ClientConfig = new SlopeLibClientConfig();
                api.Logger.Error("Failed to load client config! Falling back to default settings. Error: {0}", e);
            }

            ParseDebugColor();

            // Initialize and register the debug rendering pipeline for the opaque pass.
            // The ModSystem instance is passed to allow the renderer dynamic access to the parsed color vector.
            debugRenderer = new SlopeDebugRenderer(api, this);
            api.Event.RegisterRenderer(debugRenderer, EnumRenderStage.Opaque, "slopedebug");

            // Construct and register the interactive client-side developer command.
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
        /// Translates the configured hex color string into a normalized Vec4f suitable for OpenGL shader consumption.
        /// </summary>
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
        private TextCommandResult OnToggleDebug(TextCommandCallingArgs args)
        {
            if (debugRenderer == null) return TextCommandResult.Error("Debug renderer is not initialized.");

            // Safely cast the parsed arguments. The predefined -9999.0 values serve as sentinel 
            // indicators to determine if the user explicitly provided dimensional overrides.
            double diameter = args[0] as double? ?? -9999.0;
            double yoffset = args[1] as double? ?? -9999.0;
            bool hasArgs = diameter > -9990.0;

            // Enforce activation if explicit arguments are provided; otherwise, toggle the current state.
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
                // Ensure the local player entity possesses the behavior necessary for testing.
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior == null)
                {
                    behavior = new EntityBehaviorSlopeAware(playerEntity);
                    playerEntity.AddBehavior(behavior);
                }

                // Immediately apply explicit configurations to the active behavior instance.
                if (hasArgs)
                {
                    behavior.CollisionSphereSize = diameter;
                    behavior.CollisionSphereYOffset = yoffset > -9990.0 ? yoffset : (diameter / 2.0);

                    return TextCommandResult.Success($"SlopeLib debug view is now ON (Diameter: {behavior.CollisionSphereSize}, Y-Offset: {behavior.CollisionSphereYOffset}).");
                }
            }
            else
            {
                // Cleanly detach the behavior to halt processing overhead when debug mode is inactive.
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior != null)
                {
                    playerEntity.RemoveBehavior(behavior);
                }
            }

            return TextCommandResult.Success($"SlopeLib debug view is now {(debugRenderer.IsActive ? "ON" : "OFF")}.");
        }

        /// <summary>
        /// Executed when the mod is unloaded or the client disconnects.
        /// </summary>
        public override void Dispose()
        {
            base.Dispose();

            // CRITICAL OPTIMIZATION: We must dispose of our OpenGL MeshRef when the mod unloads 
            // to prevent VRAM memory leaks every time the player disconnects and reconnects.
            debugRenderer?.Dispose();
            debugRenderer = null;
        }
    }
}