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


    private void RunScript(bool reset, List<Line> axes, Plane rootFrame, Plane endFrame, Plane target, bool orient, DataTree<Brep> bodies, DataTree<double> range, List<Polyline> structure, int iterations, double error, double angleError, int step, ref object Axes, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object GoalPlane, ref object Solved)
    {
        // <Custom code>

        if (reset)
        {
            bot.Clear();
            Print("reset");
            bot.InitializeBot(axes, rootFrame, endFrame, bodies, range);
        }
        else
        {
            if (!bot.IsSet)
            {
                bot.InitializeBot(axes, rootFrame, endFrame, bodies, range);
            }

            if (error == 0) error = 0.1;
            if (angleError == 0) angleError = 0.01;

            if (bot.TotalIterations < iterations && !bot.TargetSolved)
            {
                if (structure.Count == 0)
                {
                    if (orient)
                    {
                        bot.Solve_IK(target, error, angleError, 5000);
                        RhinoApp.WriteLine("Solving with orient");

                    }
                    else
                    {
                        bot.Solve_IK(target, 1, 5000);
                        RhinoApp.WriteLine("Solving no orient");

                    }
                }
                else
                {
                    if (orient)
                    {
                        bot.Solve_IK(target, structure, error, angleError, iterations, true);
                        RhinoApp.WriteLine("Solving on structure");
                    }
                }
            }
            if (step >= bot.RobotSteps.Count) step = bot.RobotSteps.Count - 1;
            if (step < 0) step = 0;
            bot.goToState(bot.RobotSteps[step]);
            bot.ApplyBodies();
            if (bot.TargetSolved) Print("solve OK");
            Print("iterations = {0}", bot.Iterations);
            Print("ErrorSq = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);


            Axes = bot.RobotSteps[step].StateOrientationPlanes;
            EndPlane = bot.EndFrame;
            Bodies = bot.Bodies;
            TargetPlane = bot.RobotSteps[step].StateTargetFrame;
            GoalPlane = bot.GoalFrame;
            Solved = bot.RobotSteps[step].StateSolved;
        }
        // </Custom code>
    }

    // <Custom additional code>

    Robot bot = new Robot();

    public class Robot
    {
        private List<Line> originalAxes = new List<Line>();
        private DataTree<double> jointRange = new DataTree<double>();
        private List<Plane> originalOrientationPlanes = new List<Plane>();
        private List<Plane> reverseOriginalOrientationPlanes = new List<Plane>();
        private List<Plane> orientationPlanes = new List<Plane>();
        private List<Plane> reverseOrientationPlanes = new List<Plane>();
        private DataTree<Brep> originalBodies = new DataTree<Brep>();
        private DataTree<Brep> bodies = new DataTree<Brep>();
        private List<double> jointAngles = new List<double>();
        private List<Line> errorLines = new List<Line> { new Line(), new Line(), new Line() };
        private DataTree<RobotState> robotStates = new DataTree<RobotState>();
        private List<RobotState> robotSteps = new List<RobotState>();
        private Plane targetFrame;
        private Plane goalFrame;
        private int iterations;
        private int totalIterations;
        private int steps;
        private int tries;
        private bool targetSolved = false;
        private bool flipped = false;
        private bool goalSolved = false;
        private bool isSet = false;

        public List<Line> OriginalAxes { get { return originalAxes; } set { originalAxes = value; } }
        public DataTree<double> JointRanges { get { return jointRange; } set { jointRange = value; } }
        public List<Plane> OriginalOrientationPlanes { get { return originalOrientationPlanes; } set { originalOrientationPlanes = value; } }
        public List<Plane> OrientationPlanes { get { return orientationPlanes; } set { orientationPlanes = value; } }
        public DataTree<Brep> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Brep> Bodies { get { return bodies; } set { bodies = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public List<Line> ErrorLines { get { return errorLines; } set { errorLines = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public Plane OriginalRootFrame { get { return originalOrientationPlanes[0]; } set { originalOrientationPlanes[0] = value; } }
        public Plane RootFrame { get { return orientationPlanes[0]; } set { orientationPlanes[0] = value; } }
        public Plane OriginalEndFrame { get { return originalOrientationPlanes[originalOrientationPlanes.Count - 1]; } set { originalOrientationPlanes[originalOrientationPlanes.Count - 1] = value; } }
        public Plane EndFrame { get { return orientationPlanes[orientationPlanes.Count - 1]; } set { orientationPlanes[orientationPlanes.Count - 1] = value; } }
        public Point3d EndPoint { get { return EndFrame.Origin; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public Plane GoalFrame { get { return goalFrame; } set { goalFrame = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public int TotalIterations { get { return totalIterations; } set { totalIterations = value; } }
        public int Steps { get { return steps; } set { steps = value; } }
        public int Tries { get { return tries; } set { tries = value; } }
        public bool TargetSolved { get { return targetSolved; } set { targetSolved = value; } }
        public bool GoalSolved { get { return goalSolved; } set { goalSolved = value; } }
        public bool Flipped { get { return flipped; } set { flipped = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        public DataTree<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }
        public List<RobotState> RobotSteps { get { return robotSteps; } set { robotSteps = value; } }
        public List<object> testOut = new List<object>();
        public Robot() { }
        public class RobotState
        {
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

            public RobotState() { }
            public RobotState(Plane root, List<double> angles, List<Plane> planes, int iterations, Plane target, bool flipped, bool solved)
            {

                StateTargetFrame = target;
                StateIterations = iterations;
                StateSolved = solved;
                StateFlipped = flipped;
                foreach (Plane p in planes) StateOrientationPlanes.Add(p);
                foreach (double d in angles)
                {
                    StateJointAngles.Add(d);
                }
            }
        }
        public void InitializeBot(List<Line> axes, Plane rootPlane, Plane endPlane, DataTree<Brep> bodies, DataTree<double> jointRange)
        {
            OriginalAxes = axes;
            OriginalBodies = bodies;
            JointRanges = jointRange;

            RobotSteps.Clear();
            testOut.Clear();

            OriginalOrientationPlanes.Add(rootPlane);

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
            IsSet = true;
            RhinoApp.WriteLine("initializing robot");
        }
        public void Clear()
        {
            Bodies = OriginalBodies;
            OrientationPlanes.Clear();
            OriginalOrientationPlanes.Clear();
            JointAngles.Clear();
            ErrorLines.Clear();
            RobotSteps.Clear();
            Iterations = 0;
            TotalIterations = 0;
            Steps = 0;
            Flipped = false;
            TargetSolved = false;
            GoalSolved = false;
            IsSet = false;

        }
        public void Solve_IK(Plane target, double threshhold = 0.1, int maxIterations = 5000, bool record = false)            // Location Only
        {
            TargetFrame = target;
            threshhold = Math.Pow(threshhold, 2);
            int k = 0;
            while (Iterations < maxIterations && !TargetSolved)
            {
                stepCCD(target.Origin, true);
                if (DistError < threshhold) TargetSolved = true;
                if (record)
                {
                    if (Iterations == Math.Pow(2, k))
                    {
                        RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                        k++;
                    }
                }
            }
            if(TargetSolved) RhinoApp.WriteLine("CLOSE ENOUGH!");
        }
        public void Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000, bool record = false)            //  with orientation
        {

            TargetFrame = target;
            int k = 0;
            while (Iterations < maxIterations && !TargetSolved)
            {

                stepCCD_Orient(TargetFrame);
                if (Iterations == Math.Pow(2, k))
                {
                    RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                    k++;
                }
                if (DistError < distThreshhold && AngleError < angleThreshhold) TargetSolved = true;
                if (record) if (TargetSolved) RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
            }
        }
        public void Solve_IK(Plane target, List<Polyline> structure, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000, bool record = false)            // on Line Only
        {
            int structureMember = 0;
            while (!GoalSolved && Steps < 10 && Tries < 20)
            {
                RhinoApp.WriteLine("structureMember = {0} ... steps = {1} ... tries = {2}", structureMember, Steps, Tries);
                GoalFrame = target;
                if (record) RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                Solve_IK(GoalFrame, 1, 500); // simple solver to get close
                if (TargetSolved)
                {
                    TargetSolved = false;
                    Solve_IK(GoalFrame, distThreshhold, angleThreshhold, maxIterations); // TRY to solve final goal
                }
                Iterations = 0;
                if (TargetSolved)
                {
                    GoalSolved = true;
                    RhinoApp.WriteLine("GOAL REACHED !!!!");
                }
                else
                {
                    TargetFrame = GetTarget(ref structure, structureMember);
                    if (record) RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));

                    int k = 0;
                    RhinoApp.WriteLine("hahahahah StructureMember Before = {0}", structureMember);
                    Solve_IK(TargetFrame, 1, 500, false); // simple solver to get close                            
                    RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                    if (TargetSolved)   // if basic solver gets close enough
                    {
                        RhinoApp.WriteLine("CLOSE ENOUGH! StructureMember After = {0}", structureMember);

                        TargetSolved = false;

                        while (Iterations <= maxIterations && !TargetSolved)
                        {
                            if (Iterations % 8 == 2) targetFrame.Origin = new Line(structure[structureMember][1], structure[structureMember][2]).ClosestPoint(EndFrame.Origin, true);
                            stepCCD_Orient(TargetFrame);

                            if (record)
                            {
                                if (Iterations == Math.Pow(2, k))
                                {
                                    RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                                    k++;
                                }
                                if (DistError < distThreshhold && AngleError < angleThreshhold) TargetSolved = true;
                            }
                        }
                        RhinoApp.WriteLine("StructureMember After = {0}", structureMember);
                    }
                        if (TargetSolved)
                        {
                        if (record) RobotSteps.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, TargetSolved));
                            TargetSolved = false;
                            FlipRobot();
                            Steps++;
                            Tries = 0;
                            structureMember = 0;
                        }
                    
                    else
                    {
                        if (structureMember < structure.Count - 1) structureMember++;
                        else structureMember = 0;
                        Tries++;
                    }
                    Iterations = 0;
                }
            }
        }
        private void stepCCD(Point3d target, bool startAtEnd)     //Location Only
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
        private void stepCCD_Orient(Plane target)
        {
            switch (Iterations % 4)
            {
                case 0:           // Orient for each axis-----Z
                    if (Vector3d.VectorAngle(target.ZAxis, EndFrame.ZAxis) > 0.01)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.ZAxis, EndFrame.ZAxis);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
                case 1:          // Orient for each axis-----X
                    if (Vector3d.VectorAngle(target.XAxis, EndFrame.XAxis) > 0.01)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.XAxis, EndFrame.XAxis);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
                case 2:
                    if (DistError > 0.1)
                    {
                        for (int i = 0; i < JointAngles.Count; i++)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                        }
                    }

                    break;
                case 3:
                    if (DistError > 0.1)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
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
        public Plane GetTarget(ref List<Polyline> structure, int structureMember)
        {
            structure = structure.OrderBy(d => new Line(d[1], d[2]).DistanceTo(EndFrame.Origin, true)).ToList();
            Line strut = new Line(structure[structureMember][1], structure[structureMember][2]);
            Line structureToEnd = new Line(EndFrame.Origin, strut.ClosestPoint(EndFrame.Origin, true));
            Vector3d strutX = new Vector3d(structure[structureMember][0] - strut.ClosestPoint(structure[structureMember][0], false));
            double squareAngle = Vector3d.VectorAngle(structureToEnd.Direction, strutX, strut.Direction);
            squareAngle = squareAngle * 2 / Math.PI;
            squareAngle = Math.Round(squareAngle) - 1;

            var strutRot = Transform.Rotation(-squareAngle * Math.PI / 2, strut.Direction, strut.From);
            strutX.Transform(strutRot);
            return new Plane(strut.ClosestPoint(EndFrame.Origin, true), strutX, strut.Direction);
        }
        public void FlipRobot()         // ONLY STUFF FOR IK
        {
            RhinoApp.WriteLine("Flipping");

            Flipped = !Flipped;
            DataTree<double> rangesTemp = new DataTree<double>();

            for (int i = 0; i < JointAngles.Count; i++)
            {
                foreach (double d in JointRanges.Branch(JointRanges.Branches.Count - 1 - i)) rangesTemp.Add(d, new GH_Path(i));
                JointAngles[i] = -JointAngles[i];
            }
            OrientationPlanes.Reverse();
            JointAngles.Reverse();
            JointRanges = rangesTemp;
        }
        public void goToState(RobotState state)
        {
            OrientationPlanes = state.StateOrientationPlanes;
            Flipped = state.StateFlipped;
            TargetFrame = state.StateTargetFrame;
        }
        public void ApplyBodies()
        {
            Bodies.Clear();

            if (Flipped)
            {
                for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                {
                    var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[OrientationPlanes.Count - 1 - i]);
                    foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                    {
                        Brep brepTemp = new Brep();
                        brepTemp = b.DuplicateBrep();
                        brepTemp.Transform(reorient);
                        Bodies.Add(brepTemp, new GH_Path(i));
                    }
                }
            }
            else
            {
                for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                {
                    var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[i]);
                    foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                    {
                        Brep brepTemp = new Brep();
                        brepTemp = b.DuplicateBrep();
                        brepTemp.Transform(reorient);
                        Bodies.Add(brepTemp, new GH_Path(i));
                    }
                }
            }
        }
    }

    // </Custom additional code>
}
