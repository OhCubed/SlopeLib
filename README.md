SlopeLib

High-performance entity behavior for Vintage Story providing smooth surface normal interpolation and ground distance metrics via virtual collision sphere sampling.

Implementation

Add the slopeaware behavior to the entity configuration. Configuration is defined within the slopelib attribute object.

JSON Schema

{
  "code": "mycustomentity",
  "class": "EntityAgent",
  "behaviors": [
    {
      "code": "slopeaware",
      "slopelib": {
        "diameter": 1.0,
        "yoffset": 0.5
      }
    }
  ]
}


diameter: Sphere diameter in blocks (Default: 1.0).

yoffset: Vertical offset from entity base (Default: diameter / 2).

Data Structure

Access via entity.GetBehavior<EntityBehaviorSlopeAware>().

Properties

- SurfaceNormal (Vec3d): Normalized vector representing the interpolated surface orientation.
- DistanceToSurface (double): Orthogonal distance from the sphere boundary to the resolved plane.
- SurfacePoint (Vec3d): World coordinates of the sphere center projected onto the resolved plane.
- CollisionSphereSize (double): Current diameter of the sampling sphere.
- CollisionSphereYOffset (double): Current vertical offset of the sphere.

Debugging

/slopelib debug: Toggle visualizer.

/slopelib debug [diameter] [yoffset]: Toggle visualizer with specific dimensional overrides for the player entity.
