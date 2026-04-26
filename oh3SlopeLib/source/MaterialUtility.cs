using System;
using System.Collections.Concurrent;
using System.Linq.Expressions;
using System.Reflection;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;

namespace oh3SlopeLib
{
    /// <summary>
    /// Provides zero-allocation utilities for detecting complex block properties safely in multi-threaded contexts.
    /// </summary>
    public static class MaterialUtility
    {
        private static readonly ConcurrentDictionary<Type, System.Func<BlockEntity, int[]>> materialIdGetters
            = new ConcurrentDictionary<Type, System.Func<BlockEntity, int[]>>();

        private static readonly System.Func<BlockEntity, int[]> emptyGetter = _ => null;
        private static readonly string[] propertyNames = { "MaterialIds", "materialIds", "Materials", "materials" };

        /// <summary>
        /// Retrieves the fundamental material block of a given block, natively supporting both standard blocks and chiseled Microblocks.
        /// </summary>
        public static Block GetMaterialBlock(IBlockAccessor blockAccessor, Block block, BlockPos pos)
        {
            // Null guards to prevent NREs during asynchronous chunk loading or unloading
            if (block == null || block.Id == 0) return block;
            if (blockAccessor == null) return block;

            // Zero-allocation lock-free check for chiseled blocks / Microblocks
            if (block.EntityClass != null)
            {
                try
                {
                    BlockEntity be = blockAccessor.GetBlockEntity(pos);
                    if (be != null)
                    {
                        var getter = materialIdGetters.GetOrAdd(be.GetType(), CompileGetter);

                        if (getter != emptyGetter)
                        {
                            int[] matIds = getter(be);
                            if (matIds != null && matIds.Length > 0)
                            {
                                // The engine sorts the array by volume, so the most abundant material is always at index 0.
                                Block matBlock = blockAccessor.GetBlock(matIds[0]);
                                if (matBlock != null) return matBlock;
                            }
                        }
                    }
                }
                catch
                {
                    // Gracefully catch chunk/entity access exceptions or reflection execution failures.
                    // Silently fall back to the base block to prevent spamming the console and causing I/O lag during hot physics ticks.
                }
            }

            return block; // Fast path for standard blocks
        }

        /// <summary>
        /// Checks if a block is fundamentally made of the target material, natively supporting both standard blocks and chiseled Microblocks.
        /// </summary>
        public static bool IsMaterial(IBlockAccessor blockAccessor, Block block, BlockPos pos, EnumBlockMaterial targetMaterial)
        {
            Block matBlock = GetMaterialBlock(blockAccessor, block, pos);
            return matBlock != null && matBlock.BlockMaterial == targetMaterial;
        }

        /// <summary>
        /// Dynamically compiles a lightweight getter to prevent continuous reflection overhead.
        /// </summary>
        private static System.Func<BlockEntity, int[]> CompileGetter(Type type)
        {
            try
            {
                MemberInfo targetMember = null;

                foreach (string name in propertyNames)
                {
                    targetMember = (MemberInfo)type.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.FlattenHierarchy)
                                ?? type.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.FlattenHierarchy);

                    if (targetMember != null)
                    {
                        Type memberType = targetMember is FieldInfo f ? f.FieldType : ((PropertyInfo)targetMember).PropertyType;
                        if (memberType == typeof(int[])) break;
                        targetMember = null;
                    }
                }

                if (targetMember == null) return emptyGetter; // Safe fallback

                var instanceParam = Expression.Parameter(typeof(BlockEntity), "instance");
                var castInstance = Expression.Convert(instanceParam, type);
                Expression memberAccess = targetMember is FieldInfo field
                    ? Expression.Field(castInstance, field)
                    : Expression.Property(castInstance, (PropertyInfo)targetMember);

                return Expression.Lambda<System.Func<BlockEntity, int[]>>(memberAccess, instanceParam).Compile();
            }
            catch
            {
                // Fail gracefully if IL compilation fails due to security restrictions or unexpected architecture issues.
                // Returning emptyGetter caches the failure so we don't attempt to re-compile it on the next frame.
                return emptyGetter;
            }
        }

        /// <summary>
        /// FIX #3: Clears the dynamically compiled delegate cache. 
        /// Should be called when a game session is disposed to prevent cross-world memory leaks.
        /// </summary>
        public static void ClearCache()
        {
            materialIdGetters.Clear();
        }
    }
}