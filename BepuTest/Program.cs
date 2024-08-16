using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;

namespace BepuTest;

internal class Program
{
    static BufferPool bufferPool;
    static Simulation sim;
    static BodyHandle bodyHandle;

    static void Main(string[] args)
    {
        bufferPool = new BufferPool();
        var narrow = new NarrowPhaseCallbacks();
        var integrator = new PoseIntegratorCallbacks(new Vector3(0, -10, 0));
        var solver = new SolveDescription(8, 1);
        sim = Simulation.Create(bufferPool, narrow, integrator, solver);
        sim.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), sim.Shapes.Add(new Box(500, 1, 500))));

        var sphereShape = new Sphere(1);
        var bodyDesc = BodyDescription.CreateDynamic(new Vector3(), sphereShape.ComputeInertia(1), sim.Shapes.Add(sphereShape), 0.01f);
        bodyDesc.Pose = (new Vector3(0, 100, 0), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, 0));
        sim.Bodies.Add(bodyDesc);

        

        while (true)
        {
            Thread.Sleep((int)(1 / 60d * 1000));
            sim.Timestep(1 / 60f);
            //var pos = sim.Bodies[bodyHandle].Pose.Position;
            //Console.WriteLine(pos);
            var bodyRef = new BodyReference(bodyHandle, sim.Bodies);

            for (int s = 0; s < sim.Bodies.Sets.Length; s++)
            {
                ref var set = ref sim.Bodies.Sets[s];

                if (set.Allocated && set.Count > 0)
                {
                    for (int i = 0; i < set.Count; i++)
                    {
                        var state = set.DynamicsState[i];
                        
                        Console.WriteLine(state.Motion.Pose.Position);
                    }
                }
            }
        }
    }
}

public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
{
    public void Initialize(Simulation simulation) { }

    public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
    public readonly bool AllowSubstepsForUnconstrainedBodies => false;

    public readonly bool IntegrateVelocityForKinematics => false;

    public Vector3 Gravity;

    public PoseIntegratorCallbacks(Vector3 gravity) : this()
    {
        Gravity = gravity;
    }

    Vector3Wide gravityWideDt;

    public void PrepareForIntegration(float dt)
    {
        gravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
    }

    public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
    {
        velocity.Linear += gravityWideDt;
    }

}

struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public void Initialize(Simulation simulation)
    {
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial.FrictionCoefficient = 1f;
        pairMaterial.MaximumRecoveryVelocity = 2f;
        pairMaterial.SpringSettings = new SpringSettings(30, 1);
        return true;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }
    public void Dispose()
    {
    }
}
