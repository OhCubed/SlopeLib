using Vintagestory.API.Common;
using Vintagestory.API.Server;
using Vintagestory.API.Client;

namespace oh3SlopeLib
{
    /// <summary>
    /// Main entry point for the mod's systems.
    /// This handles registration and initialization for both server and client.
    /// </summary>
    public sealed class oh3SlopeLibModSystem : ModSystem
    {
        // Reference to the API
        private ICoreAPI api;
        private ICoreClientAPI capi;
        private SlopeDebugRenderer debugRenderer;

        public SlopeLibServerConfig ServerConfig { get; private set; }
        public SlopeLibClientConfig ClientConfig { get; private set; }

        public override void Start(ICoreAPI api)
        {
            this.api = api;
            base.Start(api);

            // Register blocks, items, entities, and behaviors here
            api.RegisterEntityBehaviorClass("slopeaware", typeof(EntityBehaviorSlopeAware));
        }

        public override void StartServerSide(ICoreServerAPI api)
        {
            base.StartServerSide(api);

            try
            {
                ServerConfig = api.LoadModConfig<SlopeLibServerConfig>("oh3SlopeLibServerConfig.json");
                if (ServerConfig == null)
                {
                    ServerConfig = new SlopeLibServerConfig();
                    api.StoreModConfig(ServerConfig, "oh3SlopeLibServerConfig.json");
                }
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
                ClientConfig = api.LoadModConfig<SlopeLibClientConfig>("oh3SlopeLibClientConfig.json");
                if (ClientConfig == null)
                {
                    ClientConfig = new SlopeLibClientConfig();
                    api.StoreModConfig(ClientConfig, "oh3SlopeLibClientConfig.json");
                }
            }
            catch (System.Exception e)
            {
                ClientConfig = new SlopeLibClientConfig();
                api.Logger.Error("Failed to load client config! Falling back to default settings. Error: {0}", e);
            }

            debugRenderer = new SlopeDebugRenderer(api, ClientConfig.DebugColor);
            api.Event.RegisterRenderer(debugRenderer, EnumRenderStage.Opaque, "slopedebug");

            // Register the client-side chat command
            api.ChatCommands.GetOrCreate("slopelib")
                .WithDescription("SlopeLib client commands")
                .BeginSubCommand("debug")
                    .WithDescription("Toggles the slope engine debug visualizer")
                    .HandleWith(OnToggleDebug)
                .EndSubCommand();
        }

        private TextCommandResult OnToggleDebug(TextCommandCallingArgs args)
        {
            if (debugRenderer == null) return TextCommandResult.Error("Debug renderer is not initialized.");

            debugRenderer.IsActive = !debugRenderer.IsActive;
            var playerEntity = capi.World.Player.Entity;

            if (debugRenderer.IsActive)
            {
                // Attach the behavior to the player if they don't already have it
                if (playerEntity.GetBehavior<EntityBehaviorSlopeAware>() == null)
                {
                    var behavior = new EntityBehaviorSlopeAware(playerEntity);
                    playerEntity.AddBehavior(behavior);
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
            // Cleanup logic if necessary
        }
    }
}