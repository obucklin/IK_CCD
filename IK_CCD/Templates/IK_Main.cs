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


    private void RunScript(bool reset, List<Line> axes, List<Line> links, Plane rootFrame, Plane endFrame, Plane target, bool orient, DataTree<Brep> bodies, DataTree<double> range, DataTree<Line> structure, int iterations, double error, double angleError, int step, ref object Axes, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object Solved)
    {
        // <Custom code>

        if (reset)
        {
            bot.Clear();
            Print("reset");
            bot.InitializeBot(axes, links, rootFrame, endFrame, bodies, range);
        }
        else
        {
            if (!bot.IsSet)
            {
                bot.InitializeBot(axes, links, rootFrame, endFrame, bodies, range);
            }

            if (error == 0) error = 0.1;
            if (angleError == 0) angleError = 0.01;

            if (bot.Iterations < iterations && !bot.Solved)
            {
                if (structure.DataCount == 0)
                {
                    if (orient)
                    {
                        bot.Solve_IK(target, error, angleError, iterations);
                        RhinoApp.WriteLine("Solving with orient");

                    }
                    else
                    {
                        bot.Solve_IK(target, 1, iterations);
                        RhinoApp.WriteLine("Solving no orient");

                    }
                }
                else
                {
                    if (orient)
                    {
                        bot.Solve_IK(target, structure, error, angleError, iterations);
                        RhinoApp.WriteLine("Solving Line");
                    }
                }
            }
            if (step >= bot.RobotSteps.Count) step = bot.RobotSteps.Count - 1;
            if (step < 0) step = 0;
            bot.goToState(bot.RobotSteps[step]);
            //Print("botsteps[step].angles[0] = {0}", bot.RobotSteps[step].JointAngles[0]);
            bot.ApplyBodies();
            if (bot.Solved) Print("solve OK");
            Print("iterations = {0}", bot.Iterations);
            Print("ErrorSq = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);
        }

        Axes = bot.OrientationPlanes;
        EndPlane = bot.EndFrame;
        Bodies = bot.Bodies;
        TargetPlane = bot.TargetFrame;
        Solved = bot.Solved;
        // </Custom code>
    }

    // <Custom additional code>

    Robot bot = new Robot();

    public class Robot
    {
        private List<Line> originalAxes = new List<Line>();
        private List<Line> axes = new List<Line>();
        private DataTree<double> jointRange = new DataTree<double>();
        private List<Plane> originalOrientationPlanes = new List<Plane>();
        private List<Plane> orientationPlanes = new List<Plane>();
        private DataTree<Brep> originalBodies = new DataTree<Brep>();
        private DataTree<Brep> bodies = new DataTree<Brep>();
        private List<Line> originalLinks = new List<Line>();
        private List<Line> links = new List<Line>();
        private List<double> jointAngles = new List<double>();
        private List<Line> errorLines = new List<Line> { new Line(), new Line(), new Line() };
        private DataTree<RobotState> robotStates = new DataTree<RobotState>();
        //private RobotState currentState = new RobotState();
        //private double distError;        //actually square of dist to shorten calculation time
        //private double angleError;
        //private Point3d endPoint;
        private Point3d targetPoint;
        private Plane targetFrame;
        private Plane goalFrame;
        private int iterations;
        private bool solved = false;
        private bool isSet = false;

        public List<Line> OriginalAxes { get { return axes; } set { axes = value; } }
        public List<Line> Axes { get { return axes; } set { axes = value; } }
        public DataTree<double> JointRange
        { get { return jointRange; } set { jointRange = value; } }
        public List<Plane> OriginalOrientationPlanes
        { get { return originalOrientationPlanes; } set { originalOrientationPlanes = value; } }
        public List<Plane> OrientationPlanes
        { get { return orientationPlanes; } set { orientationPlanes = value; } }
        public DataTree<Brep> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Brep> Bodies { get { return bodies; } set { bodies = value; } }
        public List<Line> OriginalLinks { get { return links; } set { links = value; } }
        public List<Line> Links { get { return links; } set { links = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public List<Line> ErrorLines { get { return errorLines; } set { errorLines = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public Plane OriginalRootFrame { get { return originalOrientationPlanes[0]; } set { originalOrientationPlanes[0] = value; } }
        public Plane RootFrame { get { return orientationPlanes[0]; } set { orientationPlanes[0] = value; } }
        public Plane OriginalEndFrame { get { return originalOrientationPlanes[originalOrientationPlanes.Count - 1]; } set { originalOrientationPlanes[originalOrientationPlanes.Count - 1] = value; } }
        public Plane EndFrame { get { return orientationPlanes[orientationPlanes.Count - 1]; } set { orientationPlanes[orientationPlanes.Count - 1] = value; } }
        public Point3d EndPoint { get { return EndFrame.Origin; } }

        public Point3d TargetPoint { get { return targetPoint; } set { targetPoint = value; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public Plane GoalFrame { get { return goalFrame; } set { goalFrame = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public bool Solved { get { return solved; } set { solved = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        public DataTree<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }
        public List<RobotState> RobotSteps = new List<RobotState>();
        public List<object> testOut = new List<object>();
        public Robot() { }


        public class RobotState : Robot
        {
            private Plane stateRootFrame;
            private Plane stateEndFrame;
            private int stateIterations;
            private bool stateSolved;
            private List<double> stateJointAngles = new List<double>();
            private List<Plane> stateOrientationPlanes = new List<Plane>();

            public Plane StateRootFrame { get { return stateRootFrame; } set { stateRootFrame = value; } }
            public Plane StateEndFrame { get { return stateEndFrame; } set { stateEndFrame = value; } }
            public bool StateSolved { get { return stateSolved; } set { stateSolved = value; } }
            public int StateIterations { get { return stateIterations; } set { stateIterations = value; } }
            public List<double> StateJointAngles { get { return stateJointAngles; } set { stateJointAngles = value; } }
            public List<Plane> StateOrientationPlanes { get { return stateOrientationPlanes; } set { stateOrientationPlanes = value; } }

            public RobotState() { }
            public RobotState(Plane root, List<double> angles, int iterations, bool solved)
            {

                StateRootFrame = root;
                StateIterations = iterations;
                StateSolved = solved;
                foreach (double d in angles)
                {
                    StateJointAngles.Add(d);
                }
            }

            public void SaveCurrentState()
            {
                this.RootFrame = RootFrame;
                foreach (double d in JointAngles) StateJointAngles.Add(d);
            }
        }
        public void InitializeBot(List<Line> axes, List<Line> links, Plane rootPlane, Plane endPlane, DataTree<Brep> bodies, DataTree<double> jointRange)
        {
            OriginalAxes = axes;
            OriginalLinks = links;
            OriginalBodies = bodies;
            JointRange = jointRange;

            RobotSteps.Clear();
            testOut.Clear();

            OriginalOrientationPlanes.Add(rootPlane);

            foreach (Line l in axes)
            {
                OriginalOrientationPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            OriginalOrientationPlanes.Add(endPlane);

            foreach (Plane p in OriginalOrientationPlanes) OrientationPlanes.Add(p);
            IsSet = true;
            RhinoApp.WriteLine("initializing robot");
        }
        public void Clear()
        {
            Axes = OriginalAxes;
            Bodies = OriginalBodies;
            Links = OriginalLinks;
            OrientationPlanes.Clear();
            OriginalOrientationPlanes.Clear();
            JointAngles.Clear();
            ErrorLines.Clear();
            Iterations = 0;
            Solved = false;
            IsSet = false;

        }
        public void Solve_IK(Plane target, double threshhold = 0.1, int maxIterations = 5000)            // Location Only
        {
            TargetFrame = target;
            threshhold = Math.Pow(threshhold, 2);
            int k = 0;
            while (Iterations < maxIterations && !Solved)
            {
                stepCCD(target.Origin, true);
                if (DistError < threshhold) solved = true;
                if (Iterations == Math.Pow(2, k))
                {
                    RhinoApp.WriteLine("RobotSteps count @ {1} = {0}", RobotSteps.Count, Iterations);
                    RobotSteps.Add(new RobotState(RootFrame, JointAngles, Iterations, Solved));
                    k++;
                }
            }


        }
        public void Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)            //  with orientation
        {
            TargetFrame = target;
            int k = 0;
            while (Iterations < maxIterations && !Solved)
            {
                stepCCD_Orient(TargetFrame);
                if (Iterations == Math.Pow(2, k))
                {
                    RhinoApp.WriteLine("RobotSteps count @ {1} = {0}", RobotSteps.Count, Iterations);
                    RobotSteps.Add(new RobotState(RootFrame, JointAngles, Iterations, Solved));
                    k++;
                }
                if (DistError < distThreshhold && AngleError < angleThreshhold) Solved = true;

            }
            RhinoApp.WriteLine("Fart Solved = {0}", Solved);
        }
        public void Solve_IK(Plane target, DataTree<Line> structure, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)            // on Line Only
        {
            if (!Solved)
            {
                RhinoApp.WriteLine("with structure");
                GoalFrame = target;
                targetFrame = new Plane(structure.Branch(0)[0].ClosestPoint(EndFrame.Origin, true), structure.Branch(0)[0].Direction, structure.Branch(0)[1].Direction);

                RobotSteps.Add(new RobotState(RootFrame, JointAngles, Iterations, Solved));

                int k = 0;

                while (Iterations <= maxIterations && !Solved)
                {

                    if (Iterations < 2) stepCCD(GoalFrame.Origin, false);
                    else
                    {

                        if (Iterations % 4 == 2) targetFrame.Origin = structure.Branch(0)[0].ClosestPoint(EndFrame.Origin, true);
                        stepCCD_Orient(TargetFrame);

                    }
                    if (Iterations == Math.Pow(2, k))
                    {
                        RhinoApp.WriteLine("RobotSteps count @ {1} = {0}", RobotSteps.Count, Iterations);
                        RobotSteps.Add(new RobotState(RootFrame, JointAngles, Iterations, Solved));
                        k++;
                    }
                    if (DistError < distThreshhold && AngleError < angleThreshhold) Solved = true;

                }
                if (Solved) RobotSteps.Add(new RobotState(RootFrame, JointAngles, Iterations, Solved));
                RhinoApp.WriteLine("correct angle[0] = {0}", JointAngles[0]);

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
        }

        private void stepCCD_Orient(Plane target)
        {
            switch (Iterations % 4)
            {
                case 0:           //for each axis-----Z
                    if (Vector3d.VectorAngle(target.ZAxis, EndFrame.ZAxis) > 0.01)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.ZAxis, EndFrame.ZAxis);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
                case 1:          //for each axis-----X
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
                        for (int i = 0; i < JointAngles.Count; i++)           //for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                        }
                    }

                    break;
                case 3:
                    if (DistError > 0.1)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
            }
            Iterations++;
        }

        private double getAngle(int joint, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - OrientationPlanes[joint + 1].Origin);
            Vector3d jointToGoal = new Vector3d(target - OrientationPlanes[joint + 1].Origin);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, OrientationPlanes[joint + 1]);
            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(joint))[0] != 0 && (JointRange.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRange.Branch(new GH_Path(joint))[1]))
                angle = JointRange.Branch(new GH_Path(joint))[1] - JointAngles[joint];
            return angle;
        }

        private double getAngle(int joint, Vector3d target, Vector3d end)
        {
            double angle = Vector3d.VectorAngle(end, target, OrientationPlanes[joint + 1]);

            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(joint))[0] != 0 && (JointRange.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRange.Branch(new GH_Path(joint))[1]))
                angle = JointRange.Branch(new GH_Path(joint))[1] - JointAngles[joint];

            angle = angle * (1 - Math.Abs(end * OrientationPlanes[joint + 1].Normal));
            angle = angle * (1 - Math.Abs(target * OrientationPlanes[joint + 1].Normal));

            return angle;
        }

        public void RotateJoint(int joint, double angle)
        {
            var rotate = Transform.Rotation(angle, OrientationPlanes[joint + 1].Normal, OrientationPlanes[joint + 1].Origin);

            for (int i = joint; i < OrientationPlanes.Count - 1; i++)        //move link and children
            {
                Plane plane = OrientationPlanes[i + 1];
                plane.Transform(rotate);
                OrientationPlanes[i + 1] = plane;
            }
            JointAngles[joint] += angle;
        }

        public void goToState(RobotState state)
        {
            RhinoApp.WriteLine("angle[0] = {0}", JointAngles[0]);
            RootFrame = state.StateRootFrame;
            Solved = state.StateSolved;
            var rebase = Transform.PlaneToPlane(OriginalRootFrame, state.StateRootFrame);

            for (int i = 0; i < OrientationPlanes.Count; i++)
            {
                Plane planeTemp = OriginalOrientationPlanes[i];
                planeTemp.Transform(rebase);
                OrientationPlanes[i] = planeTemp;
            }
            for (int i = 0; i < JointAngles.Count; i++)
            {
                RotateJoint(i, state.StateJointAngles[i]);
            }
        }

        public void ApplyBodies()
        {
            Bodies.Clear();
            //foreach (Brep b in OriginalBodies.Branch(new GH_Path(0)))
            //{
            //    var reorient = Transform.PlaneToPlane(OriginalRootFrame, RootFrame);
            //    Brep brepTemp = new Brep();
            //    brepTemp = b.DuplicateBrep();
            //    brepTemp.Transform(reorient);
            //    Bodies.Add(brepTemp, new GH_Path(0));
            //}
            for (int i = 0; i < OrientationPlanes.Count - 1; i++)
            {
                var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[i]);
                foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                {
                    Brep brepTemp = new Brep();
                    brepTemp = b.DuplicateBrep();
                    brepTemp.Transform(reorient);
                    Bodies.Add(brepTemp, new GH_Path(i + 1));
                    //testOut.Add(brepTemp);
                }
            }
        }
    }

    // </Custom additional code>
}
