using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom "using" statements>
using System.Linq;
using System.Diagnostics;
using System.Drawing;
using Rhino.Display;
// </Custom "using" statements>


#region padding (this ensures the line number of this file match with those in the code editor of the C# Script component






















#endregion

public partial class MyExternalScript : GH_ScriptInstance
{
    #region Do_not_modify_this_region
    private void Print(string text) { }
    private void Print(string format, params object[] args) { }
    private void Reflect(object obj) { }
    private void Reflect(object obj, string methodName) { }
    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA) { }
    public RhinoDoc RhinoDocument;
    public GH_Document GrasshopperDocument;
    public IGH_Component Component;
    public int Iteration;
    #endregion


    private void RunScript(bool reset, List<Line> axes, Plane rootFrame, Plane endFrame, Plane target, bool orient, double collisionDist, DataTree<Mesh> bodies, DataTree<double> range, List<Polyline> structure, int iterations, double error, double angleError, int step, bool animate, int path, List<double> axisControl, ref object Angles, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object GoalPlane, ref object PathPlanes, ref object Solved, ref object TestOut)
    {
        // <Custom code>

        if (reset)
        {
            bot.Clear();
            Print("reset");
            bot.InitializeBot(axes, rootFrame, endFrame, collisionDist, bodies, range, error, angleError, iterations);
            str = new Structure(structure);
            stepInternal = 0;
            material = new DisplayMaterial();
            material.Diffuse = Color.FromArgb(200, 200, 200);
        }
        else
        {
            if (str.struts.Count <1) str = new Structure(structure);
            if (!bot.IsSet)
            {
                bot.InitializeBot(axes, rootFrame, endFrame, collisionDist, bodies, range, error, angleError, iterations);
            }
            if (targetPlane.Origin != target.Origin || targetPlane.XAxis != target.XAxis || targetPlane.Normal != target.Normal)
            {
                RhinoApp.WriteLine("targetMoved");
                targetPlane = target;
                bot.Restart(false, targetPlane);
                stepInternal = 0;
            }
            if (!bot.GoalSolved && !bot.Ran)
            {
                stopwatch.Restart();
                if (structure.Count == 0)
                {
                    if (orient)
                    {
                        RhinoApp.WriteLine("Solving with orient");
                        bot.GoalSolved = bot.Solve_IK(targetPlane);
                    }
                    else
                    {
                        RhinoApp.WriteLine("Solving no orient");
                        bot.GoalSolved = bot.Solve_IK(targetPlane.Origin);
                    }
                }
                else
                {
                    if (orient)
                    {
                        bot.GoalSolved = bot.Solve_Path(targetPlane, str, true, false);
                    }
                }
                IKTime = stopwatch.ElapsedMilliseconds;
                bot.Ran = true;
                stopwatch.Restart();
                bot.Animate(bot.RobotSteps, 10);
                AnimateTime = stopwatch.ElapsedMilliseconds;

            }
            if (animate)
            {
                if (stepInternal < bot.AnimationFrames.Count - 1) stepInternal++;
            }
            else
            {
                stepInternal = step;
            }

            List<Robot.RobotState> stepsOut = new List<Robot.RobotState>(bot.RobotStates);

            if (stepInternal >= stepsOut.Count) stepInternal = stepsOut.Count - 1;
            if (stepInternal < 0) stepInternal = 0;
            if (path >= bot.Paths.Count) path = bot.Paths.Count - 1;
            if (path < 0) path = 0;
            if (stepsOut.Count > stepInternal) bot.GoToState(stepsOut[stepInternal]);

            Print("Path Count = {0}", bot.Paths.Count);
            if (bot.GoalSolved) Print("solve OK");
            Print("totalIterations = {0}", bot.TotalIterations);
            Print("IK solve time = {0}", IKTime);
            Print("Path solve time = {0}", PathTime);
            Print("Animate solve time = {0}", AnimateTime);
            Print("Error = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);

            if (structure.Count > 0)
            {
                Print("steps = {0}", bot.Steps);
                Print("frameCount = {0}", stepsOut.Count);
            }
            Print(stepsOut[stepInternal].Message + " " + stepsOut[stepInternal].StateSolved + " after iterations = " + stepsOut[stepInternal].StateIterations);
            Angles = bot.JointAngles;
            EndPlane = bot.EndFrame;
            Bodies = bot.Bodies;
            TargetPlane = bot.TargetFrame;
            if (bot.Paths[path].Count > 0) PathPlanes = bot.Paths[path].planes;
            Solved = stepsOut[stepInternal].StateSolved;
            TestOut = bot.testOut;
        }


        // </Custom code>
    }

    // <Custom additional code>


    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {

        foreach (var bodies in bot.Bodies.Branches)
        {
            foreach (Mesh body in bodies)
            {
                args.Display.DrawMeshShaded(body, material);
            }
        }
    }


    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        //args.Display.DrawPoint(new Point3d(3, 3, 3), Color.FromArgb(25, 125, 255));
    }


    public override BoundingBox ClippingBox
    {
        get
        {
            return new BoundingBox(-20000, -20000, -20000, 20000, 20000, 20000);
        }
    }

    int stepInternal = 0;
    Robot bot = new Robot();
    Structure str = new Structure();
    Plane targetPlane = new Plane();
    Stopwatch stopwatch = new Stopwatch();
    double IKTime = 0;
    double PathTime = 0;
    double AnimateTime = 0;

    DisplayMaterial material;

    public class Robot
    {
        private List<Line> originalAxes = new List<Line>();
        private DataTree<double> jointRange = new DataTree<double>();
        private List<Plane> originalOrientationPlanes = new List<Plane>();
        private List<Plane> reverseOriginalOrientationPlanes = new List<Plane>();
        private List<Plane> orientationPlanes = new List<Plane>();
        private List<Plane> reverseOrientationPlanes = new List<Plane>();
        private DataTree<Mesh> originalBodies = new DataTree<Mesh>();
        private DataTree<Mesh> bodies = new DataTree<Mesh>();
        private double collisionRadius;
        private List<double> jointAngles = new List<double>();
        private double distThreshhold;
        private double angleThreshhold;
        private List<RobotState> robotSteps = new List<RobotState>();
        private List<RobotState> robotStates = new List<RobotState>();
        private List<RobotState> animationFrames = new List<RobotState>();
        private Plane targetFrame;
        private Plane goalFrame;
        private int currentStrut;
        private int iterations;
        private int totalIterations;
        private int maxIterations;
        private int steps;
        private int tries;
        private bool stepSolved = false;
        private bool flipped = false;
        private bool goalSolved = false;
        private bool isSet = false;
        private bool ran = false;
        private bool testCollision = true;
        private List<Structure.Path> paths = new List<Structure.Path>();
        private Structure robotStructure = new Structure();

        public List<object> testOut = new List<object>();

        public List<Line> OriginalAxes { get { return originalAxes; } set { originalAxes = value; } }
        public DataTree<double> JointRanges { get { return jointRange; } set { jointRange = value; } }
        public List<Plane> OriginalOrientationPlanes { get { return originalOrientationPlanes; } set { originalOrientationPlanes = value; } }
        public List<Plane> OrientationPlanes { get { return orientationPlanes; } set { orientationPlanes = value; } }
        public DataTree<Mesh> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Mesh> Bodies
        {
            get
            {
                bodies.Clear();

                if (Flipped)
                {
                    for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                    {
                        var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[OrientationPlanes.Count - 1 - i]);
                        foreach (Mesh b in OriginalBodies.Branch(new GH_Path(i)))
                        {
                            Mesh meshTemp = new Mesh();
                            meshTemp = b.DuplicateMesh();
                            meshTemp.Transform(reorient);
                            bodies.Add(meshTemp, new GH_Path(i));
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                    {
                        var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[i]);
                        foreach (Mesh b in OriginalBodies.Branch(new GH_Path(i)))
                        {
                            Mesh meshTemp = new Mesh();
                            meshTemp = b.DuplicateMesh();
                            meshTemp.Transform(reorient);
                            bodies.Add(meshTemp, new GH_Path(i));
                        }
                    }
                }
                return bodies;
            }
            set { bodies = value; }
        }
        //public List<Line> SkeletonEnd
        //{
        //    List<Line> skeletonTemp = new List<Line>;
        //    get
        //    {
        //        Polyline skTemp = new Polyline();
        //        foreach (Plane p in orientationPlanes) skTemp.Add(p.Origin);
        //        return skTemp;
        //    }
        //}
        public double CollisionRadius { get { return collisionRadius; } set { collisionRadius = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public double NormalError { get { return Vector3d.VectorAngle(EndFrame.Normal, TargetFrame.Normal); } }
        public double DistThreshhold { get { return distThreshhold; } set { distThreshhold = value; } }
        public double AngleThreshhold { get { return angleThreshhold; } set { angleThreshhold = value; } }
        public Plane OriginalRootFrame { get { return originalOrientationPlanes[0]; } set { originalOrientationPlanes[0] = value; } }
        public Plane RootFrame { get { return orientationPlanes[0]; } set { orientationPlanes[0] = value; } }
        public Plane OriginalEndFrame { get { return originalOrientationPlanes[originalOrientationPlanes.Count - 1]; } set { originalOrientationPlanes[originalOrientationPlanes.Count - 1] = value; } }
        public Plane EndFrame { get { return orientationPlanes[orientationPlanes.Count - 1]; } set { orientationPlanes[orientationPlanes.Count - 1] = value; } }
        public Point3d EndPoint { get { return EndFrame.Origin; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public Plane GoalFrame { get { return goalFrame; } set { goalFrame = value; } }
        public int CurrentStrut { get { return currentStrut; } set { currentStrut = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public int TotalIterations { get { return totalIterations; } set { totalIterations = value; } }
        public int MaxIterations { get { return maxIterations; } set { maxIterations = value; } }
        public int Steps { get { return steps; } set { steps = value; } }
        public int Tries { get { return tries; } set { tries = value; } }
        public bool StepSolved { get { return stepSolved; } set { stepSolved = value; } }
        public bool GoalSolved { get { return goalSolved; } set { goalSolved = value; } }
        public bool Flipped { get { return flipped; } set { flipped = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        public bool Ran { get { return ran; } set { ran = value; } }
        public List<RobotState> RobotSteps { get { return robotSteps; } set { robotSteps = value; } }
        public List<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }
        public List<RobotState> AnimationFrames { get { return animationFrames; } set { animationFrames = value; } }
        public List<Structure.Path> Paths { get { return paths; } set { paths = value; } }
        public Structure RobotStructure { get { return robotStructure; }  set { robotStructure = value; } }
        public bool TestCollision { get { return testCollision; }  set { testCollision = value; } } 
        public Robot() { }
        public Robot(List<Line> axes, Plane rootPlane, Plane endPlane, double collisionRadius, DataTree<Mesh> bodies, DataTree<double> jointRange, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)
        {
            InitializeBot(axes, rootPlane, endPlane, collisionRadius, bodies, jointRange, distThreshhold, angleThreshhold, maxIterations);
        }
        public void InitializeBot(List<Line> axes, Plane rootPlane, Plane endPlane, double collisionRadius, DataTree<Mesh> bodies, DataTree<double> jointRange, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)
        {
            OriginalAxes = axes;
            OriginalBodies = bodies;
            JointRanges = jointRange;
            CollisionRadius = collisionRadius;
            RobotStates.Clear();
            RobotSteps.Clear();
            AnimationFrames.Clear();
            testOut.Clear();
            DistThreshhold = distThreshhold;
            AngleThreshhold = angleThreshhold;
            MaxIterations = maxIterations;
            OriginalOrientationPlanes.Add(rootPlane);

            if (DistThreshhold == 0) DistThreshhold = 0.1;
            if (AngleThreshhold == 0) AngleThreshhold = 0.01;
            if (MaxIterations == 0) MaxIterations = 5000;

            //TargetFrame = EndFrame;

            foreach (Line l in axes)
            {
                OriginalOrientationPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            OriginalOrientationPlanes.Add(endPlane);

            foreach (Plane p in OriginalOrientationPlanes)
            {
                OrientationPlanes.Add(p);
            }
            GoalSolved = false;
            IsSet = true;
            Ran = false;
            SaveState(ref robotStates, false, "initial position");
            SaveState(ref robotSteps, false, "initial position");
            //RhinoApp.WriteLine("initializing robot");
        }
        public void Clear()
        {
            Bodies = OriginalBodies;
            OrientationPlanes.Clear();
            OriginalOrientationPlanes.Clear();
            JointAngles.Clear();
            RobotStates.Clear();
            RobotSteps.Clear();
            AnimationFrames.Clear();
            Iterations = 0;
            TotalIterations = 0;
            Steps = 0;
            Tries = 0;
            CurrentStrut = 0;
            Flipped = false;
            StepSolved = false;
            GoalSolved = false;
            IsSet = false;
            Ran = false;
        }
        public void Restart(bool startAtEnd, Plane target)
        {
            GoalSolved = false;
            TotalIterations = 0;
            Steps = 0;
            Tries = 0;
            CurrentStrut = 0;
            Ran = false;
            if (startAtEnd) if (RobotSteps.Count > 0) GoToState(RobotSteps[RobotSteps.Count - 1]);
            RobotSteps.Clear();
            Paths.Clear();
            SaveState(ref robotSteps, true, "restart position");
            AnimationFrames.Clear();
            SaveState(ref animationFrames, true, "restart position");
            GoalFrame = target;
        }
        public bool TestDistance(Point3d target, double threshhold = 1, int maxIterations = 20)            // Location Only
        {
            bool success = false;
            while (Iterations < maxIterations && !success)
            {
                stepCCD(target, true);
                if (DistError < threshhold) success = true;
            }
            if (success) SaveState(ref robotStates, success, "testDist");
            Iterations = 0;
            return success;
        }
        public bool TestDistance(Structure.Strut strut, double threshhold = 1, int maxIterations = 50)            // Location Only
        {
            TargetFrame = strut.targetPlanes[0];
            bool success = false;
            for (int i = 0; i < 5; i++) stepCCD(strut.walkEnd, true);
            SaveState(ref robotStates, success, "try WalkEnd");

            while (Iterations < maxIterations && !success)
            {
                targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                stepCCD(TargetFrame.Origin, true);
                if (DistError < threshhold) success = true;
            }
            if (success) SaveState(ref robotStates, success, "testDistStrut");
            Iterations = 0;
            return success;
        }
        public bool Collision(int joint)
        {
            bool collision = true;
            for (int i = joint; i < OrientationPlanes.Count-1; i++)
            {
               foreach(int collisionRiskIndex in RobotStructure.struts[CurrentStrut].collisionRisks)
                {
                    if (RobotStructure.struts[collisionRiskIndex].strutLine.MinimumDistanceTo(new Line(OrientationPlanes[i].Origin, OrientationPlanes[i + 1].Origin)) < CollisionRadius) collision = true;
                }
            }
            return collision;
        }
        public bool Solve_IK(Point3d target, bool record = false, int earlyBreak = 0)            // Location Only
        {
            bool success = false;
            int stepsStart = RobotStates.Count;

            targetFrame.Origin = target;
            int k = 0;
            while (Iterations < MaxIterations && !success)
            {
                stepCCD(targetFrame.Origin);
                if (record && Iterations == Math.Pow(2, k))
                {
                    SaveState(ref robotStates, StepSolved);
                    k++;
                }
                if (earlyBreak > 0 && Iterations > earlyBreak && DistError > 1) break;
                if (DistError < DistThreshhold) success = true;
            }
            if (!record)
            {
                //RobotStates.RemoveRange(stepsStart, RobotStates.Count - stepsStart);
            }
            if (success) SaveState(ref robotStates, StepSolved, "solveIK_Point");
            Iterations = 0;
            return success;
        }
        public bool Solve_IK(Plane target, bool record = false, int earlyBreak = 0)            //  with orientation
        {
            bool success = false;
            TargetFrame = target;
            int stepsStart = RobotStates.Count;
            int k = 0;
            while (Iterations < MaxIterations && !success)
            {
                stepCCD(TargetFrame);
                if (record && Iterations == Math.Pow(2, k))
                {
                    SaveState(ref robotStates, false);
                    k++;
                }
                if (earlyBreak > 0 && Iterations > earlyBreak && (DistError > 5 || AngleError > 0.1)) break;

                if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
            }
            if (!record)
            {
                //RobotStates.RemoveRange(stepsStart, RobotStates.Count - stepsStart);
            }
            if (success) SaveState(ref robotStates, success, "solveIK_Plane");
            Iterations = 0;
            return success;
        }
        public bool Solve_IK(Structure.Strut strut, Point3d nextTarget, bool record = false, int earlyBreak = 0)            //  with orientation
        {
            var rot = Transform.Rotation(Math.PI / 2, strut.basePlane.XAxis, strut.basePlane.Origin);
            bool success = false;
            if (TestDistance(strut))
            {
                Vector3d strutToBot = new Vector3d(strut.strutLine.ClosestPoint(OrientationPlanes[1].Origin, false) - OrientationPlanes[1].Origin);
                strutToBot.Unitize();
                Vector3d strutToNextTarget = new Vector3d(strut.strutLine.ClosestPoint(nextTarget, false) - nextTarget);
                strutToNextTarget.Unitize();
                List<Plane> targets = new List<Plane>();
                foreach (Plane p in strut.targetPlanes) targets.Add(new Plane(p));
                targets = targets.OrderBy(v => Vector3d.VectorAngle(new Vector3d(strutToBot + strutToNextTarget), v.Normal)).ToList();
                int rotations = 0;
                while (!success && rotations < 4)
                {
                    TargetFrame = targets[rotations];
                    targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                    int k = 0;
                    while (Iterations < MaxIterations && !success)
                    {
                        if (Iterations % 16 == 14) targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                        stepCCD(TargetFrame);
                        if (record && Iterations == Math.Pow(2, k))
                        {
                            SaveState(ref robotStates, false);
                            k++;
                        }
                        if (earlyBreak > 0 && Iterations == ((earlyBreak * 4) - 1) && (DistError > 10 || NormalError > 0.2)) break;
                        if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
                    }
                    rotations++;
                    SaveState(ref robotStates, success, "solveIK_Strut");
                    Iterations = 0;
                }
            }
            if (success) SaveState(ref robotStates, success, "solveIK_Strut");
            Iterations = 0;
            return success;
        }

        public bool GrabStrut(Structure.Strut strut, Point3d nextTarget, bool offsetSteps, bool record = false, int earlyBreak = 0)            //  with orientation
        {
            bool success = false;
            if (TestDistance(strut))
            {
                Vector3d strutToBot = new Vector3d(strut.strutLine.ClosestPoint(OrientationPlanes[1].Origin, false) - OrientationPlanes[1].Origin);
                strutToBot.Unitize();
                Vector3d strutToNextTarget = new Vector3d(strut.strutLine.ClosestPoint(nextTarget, false) - nextTarget);
                strutToNextTarget.Unitize();
                List<Plane> targets = new List<Plane>();
                foreach (Plane p in strut.targetPlanes) targets.Add(new Plane(p));
                targets = targets.OrderBy(v => Vector3d.VectorAngle(new Vector3d(strutToBot + strutToNextTarget), v.Normal)).ToList();
                int rotations = 0;
                while (!success && rotations < 4)           //try each side of strut
                {
                    TargetFrame = targets[rotations];
                    Line offsetLine = new Line(strut.strutLine.From - (targets[rotations].Normal) *1.5* CollisionRadius, strut.strutLine.To - (targets[rotations].Normal) * CollisionRadius);
                    if (offsetSteps) targetFrame.Origin = offsetLine.ClosestPoint(EndFrame.Origin, true);
                    else targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                    int k = 0;
                    while (Iterations < MaxIterations && !success)
                    {
                        if (Iterations % 16 == 14)
                        {
                            if (offsetSteps) targetFrame.Origin = offsetLine.ClosestPoint(EndFrame.Origin, true);
                            else targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                        }
                        stepCCD(TargetFrame);
                        if (record && Iterations == Math.Pow(2, k))
                        {
                            SaveState(ref robotStates, false);
                            k++;
                        }
                        if (earlyBreak > 0 && Iterations == ((earlyBreak * 4) - 1) && (DistError > 10 || NormalError > 0.2)) break;
                        if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
                    }
                    if (!success) rotations++;
                    SaveState(ref robotSteps, success, "solveIK_StrutOFFSETS");
                    Iterations = 0;
                }
                if (success && offsetSteps)
                {
                    targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                    success = false;
                    offsetSteps = false;

                    while (Iterations < MaxIterations && !success)
                    {
                        if (Iterations % 16 == 14)
                        {
                            targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                        }
                        stepCCD(TargetFrame);
                        if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
                    }
                    offsetSteps = true;

                    rotations++;
                    SaveState(ref robotSteps, success, "solveIK_ONStrut");
                    Iterations = 0;
                }
            }
            Iterations = 0;
            return success;
        }



        public bool Solve_Path(Plane target, Structure structure, bool offsetSteps, bool record = false)            // on Line Only
        {

            RhinoApp.WriteLine("Solving Path");
            Stopwatch sw = new Stopwatch();
            sw.Restart();
            RobotStructure = structure;
            Paths = RobotStructure.GetPaths(RootFrame, GoalFrame, 200, 360);
            RhinoApp.WriteLine("Path Solver Time = {0}", sw.ElapsedMilliseconds);

            bool success = false;
            GoalFrame = target;
            int i = 0;
            sw.Restart();
            foreach (Structure.Path path in paths)
            {
                i++;
                foreach (Structure.Strut s in path.struts) testOut.Add(s.walkEnd);
                while (CurrentStrut < path.struts.Count && Steps < 50 && Tries < 50)         // try to reach goal following path
                {
                    if (CurrentStrut < path.struts.Count - 1)                                //if not on last strut
                    {
                        StepSolved = GrabStrut(path.struts[CurrentStrut + 1], OrientationPlanes[OrientationPlanes.Count - 2].Origin, offsetSteps, false, 200);
                        if (StepSolved)         //try to reach next strut
                        {
                            CurrentStrut++;
                            SaveState(ref robotSteps, StepSolved, "solveIK_Strut");
                            FlipRobot();

                        }
                        else        //else step on this strut
                        {
                            StepSolved = GrabStrut(path.struts[CurrentStrut], path.struts[CurrentStrut + 1].walkEnd, offsetSteps, false, 200);
                            if (StepSolved)
                            {
                                SaveState(ref robotSteps, StepSolved, "solveIK_Strut");
                                FlipRobot();
                            }
                            else Tries++;
                        }
                    }
                    else                                                        //if on last strut
                    {
                        GoalSolved = Solve_IK(GoalFrame, false);
                        StepSolved = GoalSolved;
                        if (GoalSolved)
                        {
                            SaveState(ref robotSteps, true, "solveIK_Goal");
                            break;                                         //if not solved, go to nex path
                        }
                        else
                        {
                            StepSolved = GrabStrut(path.struts[CurrentStrut], goalFrame.Origin, offsetSteps, false, 200);        //else step on this strut
                            if (StepSolved)
                            {
                                SaveState(ref robotSteps, StepSolved, "solveIK_Strut");
                                FlipRobot();
                            }
                            else Tries++;
                        }
                    }
                }
                if (GoalSolved)
                {
                    RhinoApp.WriteLine("SOLVED!! YAY!");
                    success = GoalSolved;
                    break;                                         //if not solved, go to nex path
                }
                else
                {
                    CurrentStrut = 0;
                    Tries = 0;
                }
            }
            RhinoApp.WriteLine("IK Solver Time = {0}", sw.ElapsedMilliseconds);
            if (!GoalSolved) Solve_IK(GoalFrame, false);
            return success;
        }
        private void stepCCD(Point3d target, bool startAtEnd = true)     //Location Only
        {
            if (startAtEnd)
            {
                for (int i = JointAngles.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                {
                    double angle = getAngle(i, target, EndFrame.Origin);
                    RotateJoint(i, angle);
                }
            }
            else
            {
                for (int i = 0; i < JointAngles.Count; i++)           //for each axis-----ORIGIN  
                {
                    double angle = getAngle(i, target, EndFrame.Origin);
                    RotateJoint(i, angle);
                }
            }
            Iterations++;
            TotalIterations++;
        }
        private void stepCCD(Plane target)
        {
            switch (Iterations % 4)
            {
                case 0:           // Orient for each axis-----Z
                    if (AngleError > AngleThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.ZAxis, EndFrame.ZAxis);
                            RotateJoint(i, angle);
                            if (TestCollision)
                            {
                                SaveState(ref robotStates, false, "bump");
                                if (Collision(i)) RotateJoint(i, -angle);
                                SaveState(ref robotStates, false, "de-bump");
                            }
                        }
                    }
                    break;
                case 1:          // Orient for each axis-----X
                    if (AngleError > AngleThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.XAxis, EndFrame.XAxis);
                            RotateJoint(i, angle);
                            if (TestCollision)
                            {
                                SaveState(ref robotStates, false, "bump");
                                if (Collision(i)) RotateJoint(i, -angle);
                                SaveState(ref robotStates, false, "de-bump");
                            }
                        }
                    }
                    break;
                case 2:
                    if (DistError > DistThreshhold)
                    {
                        for (int i = 0; i < JointAngles.Count; i++)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                            if (TestCollision)
                            {
                                SaveState(ref robotStates, false, "bump");
                                if (Collision(i)) RotateJoint(i, -angle);
                                SaveState(ref robotStates, false, "de-bump");
                            }
                        }
                    }

                    break;
                case 3:
                    if (DistError > DistThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                            if (TestCollision)
                            {
                                SaveState(ref robotStates, false, "bump");
                                if (Collision(i)) RotateJoint(i, -angle);
                                SaveState(ref robotStates, false, "de-bump");

                            }
                        }
                    }
                    break;
            }
            Iterations++;
            TotalIterations++;
        }
        private double getAngle(int joint, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - OrientationPlanes[joint + 1].Origin);
            Vector3d jointToGoal = new Vector3d(target - OrientationPlanes[joint + 1].Origin);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, OrientationPlanes[joint + 1]);
            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRanges.Branch(new GH_Path(joint))[0] != 0 && (JointRanges.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRanges.Branch(new GH_Path(joint))[1]))
                angle = JointRanges.Branch(new GH_Path(joint))[1] - JointAngles[joint];
            return angle;
        }
        private double getAngle(int joint, Vector3d target, Vector3d end)
        {
            double angle = Vector3d.VectorAngle(end, target, OrientationPlanes[joint + 1]);

            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRanges.Branch(new GH_Path(joint))[0] != 0 && (JointRanges.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRanges.Branch(new GH_Path(joint))[1]))
                angle = JointRanges.Branch(new GH_Path(joint))[1] - JointAngles[joint];

            angle = angle * (1 - Math.Abs(end * OrientationPlanes[joint + 1].Normal));
            angle = angle * (1 - Math.Abs(target * OrientationPlanes[joint + 1].Normal));


            return angle;
        }

        //public void getAngleAvoid(int joint, Point3d target, Point3d end)
        //{
        //    for (int i = joint; i < Skeleton.Count; i++)
        //    {

        //    }
        //}
        public void RotateJoint(int joint, double angle)
        {
            var rotate = Transform.Rotation(angle, OrientationPlanes[joint + 1].Normal, OrientationPlanes[joint + 1].Origin);
            if (Flipped)
            {
                for (int i = joint; i < OrientationPlanes.Count - 2; i++)        //move link and children
                {
                    Plane plane = OrientationPlanes[i + 2];
                    plane.Transform(rotate);
                    OrientationPlanes[i + 2] = plane;
                }
            }
            else
            {
                for (int i = joint; i < OrientationPlanes.Count - 1; i++)        //move link and children
                {
                    Plane plane = OrientationPlanes[i + 1];
                    plane.Transform(rotate);
                    OrientationPlanes[i + 1] = plane;
                }
            }
            JointAngles[joint] += angle;
        }
        public void FlipRobot()         // ONLY STUFF FOR IK
        {
            Flipped = !Flipped;
            DataTree<double> rangesTemp = new DataTree<double>();

            for (int i = 0; i < JointAngles.Count; i++)
            {
                foreach (double d in JointRanges.Branch(JointRanges.Branches.Count - 1 - i)) rangesTemp.Add(d, new GH_Path(i));
                JointAngles[i] = -JointAngles[i];
            }
            OrientationPlanes.Reverse();
            StepSolved = false;
            JointAngles.Reverse();
            JointRanges = rangesTemp;
            Steps++;
        }
        public void SaveState(ref List<RobotState> list, bool solved, string msg = "")
        {
            list.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, Iterations, TargetFrame, Flipped, solved, msg));
        }
        public void Animate(List<RobotState> states, int substates)
        {
            Robot animateBot = new Robot(OriginalAxes, OriginalRootFrame, OriginalEndFrame, CollisionRadius, OriginalBodies, JointRanges);
            animateBot.GoToState(RobotSteps[0]);
            for (int i = 0; i < states.Count - 1; i++)
            {
                animateBot.SaveState(ref animationFrames, false);
                List<double> stepSizes = new List<double>();
                for (int k = 0; k < animateBot.JointAngles.Count; k++) stepSizes.Add((states[i + 1].StateJointAngles[k] % (2 * Math.PI) - animateBot.JointAngles[k] % (2 * Math.PI)) / substates);
                for (int j = 0; j < substates; j++)
                {
                    for (int k = 0; k < animateBot.JointAngles.Count; k++)
                    {
                        animateBot.RotateJoint(k, stepSizes[k]);
                    }
                    animateBot.SaveState(ref animationFrames, false);
                }
                animateBot.FlipRobot();
            }
        }
        public void GoToState(RobotState state)
        {
            Flipped = state.StateFlipped;
            TargetFrame = new Plane(state.StateTargetFrame);
            for (int i = 0; i < state.StateOrientationPlanes.Count; i++) OrientationPlanes[i] = new Plane(state.StateOrientationPlanes[i]);
            for (int i = 0; i < state.StateJointAngles.Count; i++) JointAngles[i] = state.StateJointAngles[i];
        }
        public class RobotState
        {
            private String message;
            private Plane stateTargetFrame;
            private int stateIterations;
            private bool stateSolved;
            private bool stateFlipped;
            private List<double> stateJointAngles = new List<double>();
            private List<Plane> stateOrientationPlanes = new List<Plane>();

            public Plane StateRootFrame { get { return stateOrientationPlanes[0]; } set { stateOrientationPlanes[0] = value; } }
            public Plane StateTargetFrame { get { return stateTargetFrame; } set { stateTargetFrame = value; } }
            public bool StateSolved { get { return stateSolved; } set { stateSolved = value; } }
            public bool StateFlipped { get { return stateFlipped; } set { stateFlipped = value; } }
            public int StateIterations { get { return stateIterations; } set { stateIterations = value; } }
            public List<double> StateJointAngles { get { return stateJointAngles; } set { stateJointAngles = value; } }
            public List<Plane> StateOrientationPlanes { get { return stateOrientationPlanes; } set { stateOrientationPlanes = value; } }
            public string Message { get { return message; } set { message = value; } }

            public RobotState() { }
            public RobotState(Plane root, List<double> angles, List<Plane> planes, int iterations, Plane target, bool flipped, bool solved, string msg)
            {
                StateTargetFrame = new Plane(target);
                StateIterations = iterations;
                StateSolved = solved;
                StateFlipped = flipped;
                foreach (Plane p in planes) StateOrientationPlanes.Add(new Plane(p));
                foreach (double d in angles) StateJointAngles.Add(d);
                Message = msg;
            }
        }
    }

    public class Structure
    {
        public List<Polyline> structure = new List<Polyline>();
        public List<Strut> struts = new List<Strut>();
        public List<object> testOut = new List<object>();
        public List<List<int>> neighborTree = new List<List<int>>();

        public Structure()
        {
        }
        public Structure(List<Polyline> inputCurves)
        {
            for (int i = 0; i < inputCurves.Count; i++)
            {
                struts.Add(new Strut(inputCurves[i], i));
            }
            foreach (Strut strut in struts)
            {
                strut.getNeighbors(200, 360, struts);
                neighborTree.Add(strut.neighbors);
            }

        }

        public void printListList(List<List<int>> data)
        {
            for (int i = 0; i < data.Count; i++)
            {
                RhinoApp.Write("path {0}:", i);

                foreach (int j in data[i])
                {
                    RhinoApp.Write(j.ToString());
                    RhinoApp.Write(" , ");
                }
                RhinoApp.WriteLine();
            }
        }

        public List<Path> GetPaths(Plane startPlane, Plane targetPlane, double maxDistance, double collisionDist)
        {
            List<Path> Paths = new List<Path>();
            int startIndex = struts.OrderBy(s => s.strutLine.DistanceTo(startPlane.Origin, true)).ToList()[0].index;

            struts[startIndex].walkStart = struts[startIndex].strutLine.ClosestPoint(startPlane.Origin, true);
            int targetIndex = struts.OrderBy(s => s.strutLine.DistanceTo(targetPlane.Origin, true)).ToList()[0].index;

            if (startIndex == targetIndex)
            {
                Paths.Add(new Path());
                Paths[0].Add(struts[targetIndex]);
            }
            else
            {

                List<List<int>> pathsOut = new List<List<int>>();


                pathsOut.Add(new List<int>());
                pathsOut[0].Add(startIndex);

                int breaker = 0;
                int currentPath = 0;
                do
                {
                    bool connected = false;
                    bool deadEnd = false;
                    while (!connected && !deadEnd)  // keep working on same path
                    {
                        if (pathsOut[currentPath].Last() == targetIndex)
                        {
                            connected = true;
                            currentPath++;
                            break;
                        }
                        bool oneDone = false;
                        foreach (int testIndex in neighborTree[pathsOut[currentPath].Last()])
                        {
                            if (!pathsOut[currentPath].Contains(testIndex))                //no looping
                            {
                                if (!oneDone)                                    //first one, add to current path
                                {
                                    pathsOut[currentPath].Add(testIndex);
                                    oneDone = true;
                                }
                                else                                        // not first one... make new path
                                {
                                    List<int> pathTemp = new List<int>(pathsOut[currentPath]);
                                    pathTemp[pathTemp.Count - 1] = testIndex;
                                    pathsOut.Add(new List<int>(pathTemp));
                                }
                            }
                        }
                        if (!oneDone && !connected)                         //dead end
                        {
                            deadEnd = true;
                            pathsOut.RemoveAt(currentPath);
                            break;
                        }
                    }
                    breaker++;
                } while (pathsOut.Count > currentPath && breaker < 100);

                pathsOut = pathsOut.OrderBy(l => l.Count).ToList();
                printListList(pathsOut);
                foreach (List<int> path in pathsOut)
                {
                    Path thisPath = new Path();
                    foreach (int i in path)
                    {
                        thisPath.Add(new Strut(struts[i]));
                    }
                    Paths.Add(thisPath);
                }



                foreach (Path path in Paths)
                {
                    for (int i = 0; i < path.Count - 1; i++)
                    {
                        Point3d thisPoint = new Point3d();
                        Point3d nextPoint = new Point3d();
                        LineCurve thisLine = new LineCurve(path.struts[i].strutLine);
                        LineCurve nextLine = new LineCurve(path.struts[i + 1].strutLine);
                        thisLine.ClosestPoints(nextLine, out thisPoint, out nextPoint);
                        path.struts[i].walkEnd = thisPoint;
                    }
                    path.struts[path.struts.Count - 1].walkEnd = path.struts[path.struts.Count - 1].strutLine.ClosestPoint(targetPlane.Origin, true);
                }
            }
            return Paths;
        }

        public class Strut : Structure
        {
            public Plane basePlane;
            public double length;
            public Line strutLine;
            public int index;
            public Point3d walkStart;
            public Point3d walkEnd;
            public List<Plane> targetPlanes = new List<Plane>();
            public List<int> neighbors = new List<int>();
            public List<int> collisionRisks = new List<int>();

            public Strut()
            {
            }
            public Strut(Strut strutIn)
            {
                basePlane = new Plane(strutIn.basePlane);
                length = strutIn.length;
                strutLine = strutIn.strutLine;
                index = strutIn.index;
                walkStart = new Point3d(strutIn.walkStart);
                walkEnd = new Point3d(strutIn.walkEnd);
                foreach (Plane p in strutIn.targetPlanes) targetPlanes.Add(new Plane(p));
            }
            public Strut(Polyline inputCurve, int indexIn)
            {
                Point3d origin = inputCurve[1];
                Point3d strutEnd;
                Point3d yPoint;


                if (origin.DistanceTo(inputCurve[0]) > origin.DistanceTo(inputCurve[2]))
                {
                    strutEnd = inputCurve[0];
                    yPoint = inputCurve[2];
                }
                else
                {
                    strutEnd = inputCurve[2];
                    yPoint = inputCurve[0];
                }
                basePlane = new Plane(origin, strutEnd, yPoint);
                length = origin.DistanceTo(strutEnd);
                strutLine = new Line(origin, strutEnd);
                index = indexIn;
                for (int i = 0; i < 4; i++)
                {
                    Plane baseTemp = new Plane(basePlane);
                    var rot = Transform.Rotation(i * Math.PI / 2, basePlane.XAxis, basePlane.Origin);
                    baseTemp.Transform(rot);
                    targetPlanes.Add(baseTemp);
                }
            }

            public void getNeighbors(double maxDistance, double collisionDist, List<Strut> struts)
            {
                LineCurve thisLine = new LineCurve(strutLine);
                foreach (Strut strut in struts)
                {
                    if (index != strut.index && !collisionRisks.Contains(strut.index) && !neighbors.Contains(strut.index))
                    {
                        Point3d thisPoint = new Point3d();
                        Point3d nextPoint = new Point3d();
                        LineCurve nextLine = new LineCurve(strut.strutLine);
                        thisLine.ClosestPoints(nextLine, out thisPoint, out nextPoint);
                        double dist = thisPoint.DistanceTo(nextPoint);

                        if (dist < maxDistance)
                        {
                            neighbors.Add(strut.index);
                        }
                        if (dist < collisionDist)
                        {
                            collisionRisks.Add(strut.index);
                        }
                    }
                }
            }
        }
        public class Path
        {
            public List<Strut> struts = new List<Strut>();
            public List<int> indexList = new List<int>();
            public List<Plane> planes = new List<Plane>();
            public Strut endStrut { get { return struts.Last(); } }
            public int Count { get { return struts.Count; } }
            public List<Line> connectorLines
            {
                get
                {
                    List<Line> linestemp = new List<Line>();
                    for (int i = 0; i < struts.Count - 1; i++)
                    {
                        Line cl = new Line(struts[i].walkEnd, struts[i + 1].walkStart);
                        linestemp.Add(cl);
                    }
                    return linestemp;
                }
            }

            public Path() { }

            public void Add(Strut strut)
            {
                struts.Add(strut);
                indexList.Add(strut.index);
                planes.Add(strut.basePlane);
            }

            public void PrintPath()
            {
                foreach (Strut s in struts) RhinoApp.Write("{0}..", s.index);
                RhinoApp.WriteLine("");
            }
        }
    }



    // </Custom additional code>
}
