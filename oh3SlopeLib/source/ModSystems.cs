using Vintagestory.API.Common;
using Vintagestory.API.Server;
using Vintagestory.API.Client;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Main entry point for the mod's systems.
    /// This handles registration and initialization for both server and client.
    /// </summary>
    public sealed class oh3SlopeLibModSystem : ModSystem
    {
        // Reference to the API
        private ICoreClientAPI capi;
        private SlopeDebugRenderer debugRenderer;

        public SlopeLibServerConfig ServerConfig { get; private set; }
        public SlopeLibClientConfig ClientConfig { get; private set; }

        // Stores the parsed config color for the shader to use
        public Vec4f DebugColorVec { get; private set; } = new Vec4f(0f, 1f, 1f, 1f);

        public override void Start(ICoreAPI api)
        {
            base.Start(api);

            // Register blocks, items, entities, and behaviors here
            api.RegisterEntityBehaviorClass("slopeaware", typeof(EntityBehaviorSlopeAware));
        }

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

            // Pass the ModSystem itself so the renderer can access DebugColorVec
            debugRenderer = new SlopeDebugRenderer(api, this);
            api.Event.RegisterRenderer(debugRenderer, EnumRenderStage.Opaque, "slopedebug");

            // Register the client-side chat command using the new API
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
                catch { /* Keep default cyan on parse failure */ }
            }
        }

        private TextCommandResult OnToggleDebug(TextCommandCallingArgs args)
        {
            if (debugRenderer == null) return TextCommandResult.Error("Debug renderer is not initialized.");

            // Safely cast the parsed objects. If the user doesn't enter anything, our -9999.0 defaults act as missing flags
            double diameter = args[0] as double? ?? -9999.0;
            double yoffset = args[1] as double? ?? -9999.0;
            bool hasArgs = diameter > -9990.0;

            // If the user provided arguments, force the debug view ON instead of toggling
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
                // Attach the behavior to the player if they don't already have it
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior == null)
                {
                    behavior = new EntityBehaviorSlopeAware(playerEntity);
                    playerEntity.AddBehavior(behavior);
                }

                // If values were passed in the command, instantly apply them to the behavior!
                if (hasArgs)
                {
                    behavior.CollisionSphereSize = diameter;
                    behavior.CollisionSphereYOffset = yoffset > -9990.0 ? yoffset : (diameter / 2.0);

                    return TextCommandResult.Success($"SlopeLib debug view is now ON (Diameter: {behavior.CollisionSphereSize}, Y-Offset: {behavior.CollisionSphereYOffset}).");
                }
            }
            else
            {
                // Remove the behavior when debug mode is disabled
                var behavior = playerEntity.GetBehavior<EntityBehaviorSlopeAware>();
                if (behavior != null)
                {
                    playerEntity.RemoveBehavior(behavior);
                }
            }

            return TextCommandResult.Success($"SlopeLib debug view is now {(debugRenderer.IsActive ? "ON" : "OFF")}.");
        }

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