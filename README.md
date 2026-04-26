#SlopeLib
High-performance entity behavior for Vintage Story providing smooth surface normal interpolation and ground distance metrics via virtual collision sphere sampling.

#Implementation
Add the `slopeaware` behavior to the entity configuration. Configuration is defined within the `slopelib` attribute object.
To support complex multi-point physics (like a 4-wheeled vehicle), `slopelib` accepts an array of sphere definitions. (A single object is also supported for simple entities).

#JSON Schema
```
"behaviors": [
  {
    "code": "slopeaware",
    "slopelib": [
      {
        "diameter": 1.0,
        "xoffset": 0.0,
        "yoffset": 0.5,
        "zoffset": 0.0
      }
    ]
  }
]
```

- `diameter`: Sphere diameter in blocks (Default: 1.0).
- `xoffset`: X-axis offset relative to the entity center (Default: 0.0).
- `yoffset`: Vertical offset from entity base (Default: diameter / 2).
- `zoffset`: Z-axis offset relative to the entity center (Default: 0.0).

#Data Structure

Access the behavior on the active entity:
`var slopeBehavior = entity.GetBehavior<EntityBehaviorSlopeAware>();`

#Properties

`SurfaceDataList` (`SurfaceData[]`)
- An array of objects representing the live state of every configured sampling sphere.
Each `SurfaceData` object contains:
- `CollisionSphereSize` (`double`): Current diameter of the sampling sphere.
- `CollisionSphereXOffset` (`double`): Current X-axis offset.
- `CollisionSphereYOffset` (`double`): Current vertical offset.
- `CollisionSphereZOffset` (`double`): Current Z-axis offset.
- `Surfaces` (`TrackedSurface[4]`): An array of the 4 closest distinctly separate geometric planes detected around this sphere, sorted by proximity (Index 0 is always the closest).

Each `TrackedSurface` object contains:
- `SurfaceNormal` (`Vec3d`): Normalized vector representing the interpolated surface orientation.
- `DistanceToSurface` (`double`): Orthogonal distance from the sphere boundary to the resolved plane.
- `SurfacePoint` (`Vec3d`): World coordinates of the point on the resolved terrain plane closest to the sphere center. Useful as a plane origin for physics calculations.

#API Methods
- `UpdateSphereConfig(int index, double size, double xOffset, double yOffset, double zOffset)`
Dynamically mutates the geometric configuration of a specific sampling sphere in real-time. Designed to be called continuously from animation/physics loops without triggering Garbage Collection allocations.
- `GetPhysicalSurfaceBlock(IBlockAccessor blockAccessor, Vec3d normal, double pX, double pY, double pZ, double maxDepth = 0.8)`
Emulates a zero-allocation raycast along an inverted surface normal to find the physical block backing the mathematical plane. Natively resolves Microblock/Chiseled materials. Crucial for determining material types (e.g. ice, dirt) for friction mechanics.

#Debugging
- `.slopelib debug`
Toggles the visualizer. Renders glowing planes representing all tracked distinct surface normals for all configured spheres, accompanied by a real-time tracking HUD.
- `.slopelib debug [diameter] [yoffset]`
Toggles the visualizer with specific dimensional overrides for the primary sampling sphere on the player entity.
