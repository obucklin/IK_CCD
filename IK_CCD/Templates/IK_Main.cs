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


    private void RunScript(bool reset, List<Line> axes, Plane target, bool orient, DataTree<Brep> bodies, DataTree<double> range, DataTree<Line> structure, int iterations, double error, double angleError, ref object Axes, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object Solved)
    {
        // <Custom code>

        bool solvedTemp = false;
        if (reset)
        {
            bot.Clear();
            Print("reset");
        }
        else
        {
            if (!bot.IsSet)
            {

                bot.InitializeBot(axes, bodies, range);
            }
            if (error == 0) error = 0.1;
            if (angleError == 0) angleError = 0.01;

            if (structure.DataCount == 0)
            {
                if (orient)
                {
                    RhinoApp.WriteLine("Solving with orient");
                    solvedTemp = bot.Solve_IK(target, error, angleError, iterations);
                }
                else
                {
                    RhinoApp.WriteLine("Solving no orient");
                    solvedTemp = bot.Solve_IK(target.Origin, 1, iterations);
                }
            }
            else
            {
                if (orient)
                {
                    RhinoApp.WriteLine("Solving Line");
                    solvedTemp = bot.Solve_IK(target, structure, error, angleError, iterations);
                }
            }


            bot.ApplyBodies();
            if (solvedTemp) Print("solve OK");
            Print("iterations = {0}", bot.Iterations);
            Print("ErrorSq = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);
        }
        //bot.Iterations = 0;
        Axes = bot.Links;
        EndPlane = bot.EndFrame;
        Bodies = bot.Bodies;
        TargetPlane = bot.TargetFrame;
        Solved = solvedTemp;
        // </Custom code>
    }

    // <Custom additional code>

    Robot bot = new Robot();

    public class Robot
    {
        private List<Line> originalAxes = new List<Line>();
        private List<Line> axes = new List<Line>();
        private DataTree<double> jointRange = new DataTree<double>();
        private List<Plane> originalAxisPlanes = new List<Plane>();
        private List<Plane> axisPlanes = new List<Plane>();
        private DataTree<Brep> originalBodies = new DataTree<Brep>();
        private DataTree<Brep> bodies = new DataTree<Brep>();
        private List<Line> originalLinks = new List<Line>();
        private List<Line> links = new List<Line>();
        private List<double> jointAngles = new List<double>();
        //private List<RobotState> robotStates = new List<RobotState>();
        //private double distError;        //actually square of dist to shorten calculation time
        //private double angleError;
        //private Point3d endPoint;
        private Plane endFrame;
        private Plane originalEndFrame;
        private Point3d targetPoint;
        private Plane targetFrame;
        private Plane targetPlane;
        private int iterations;
        private bool isSet = false;

        public List<Line> OriginalAxes { get { return axes; } set { axes = value; } }
        public List<Line> Axes { get { return axes; } set { axes = value; } }
        public List<Line> OriginalLinks { get { return originalLinks; } set { originalLinks = value; } }
        public List<Line> Links { get { return links; } set { links = value; } }
        public List<Plane> OriginalAxisPlanes { get { return originalAxisPlanes; } set { originalAxisPlanes = value; } }
        public List<Plane> AxisPlanes { get { return axisPlanes; } set { axisPlanes = value; } }
        public Plane OriginalEndFrame { get { return originalEndFrame; } set { originalEndFrame = value; } }
        public Plane EndFrame { get { return endFrame; } set { endFrame = value; } }
        public Point3d EndPoint { get { return endFrame.Origin; } }
        public DataTree<Brep> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Brep> Bodies { get { return bodies; } set { bodies = value; } }

        public DataTree<double> JointRange { get { return jointRange; } set { jointRange = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }

        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public double TargetDistance { get { return Math.Abs(EndPoint.X - TargetPlane.Origin.X) + Math.Abs(EndPoint.Y - TargetPlane.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }

        public Point3d TargetPoint { get { return targetPoint; } set { targetPoint = value; } }
        public Plane TargetPlane { get { return targetPlane; } set { targetPlane = value; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }

        public int Iterations { get { return iterations; } set { iterations = value; } }
        struct robotState
        {
            public List<double> JointAnglesTemp;
            public double TargetDistanceTemp;

            public robotState(double dist)
            {
                TargetDistanceTemp = dist;
                JointAnglesTemp = new List<double>();
            }
        }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        //public List<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }


        public Robot() { }
        public Robot(List<Line> axes, DataTree<double> jointRange)           //bare skeleton
        {
            OriginalAxes = axes;
            JointRange = jointRange;

            for (int i = 0; i < OriginalAxes.Count - 1; i++)
            {
                OriginalLinks.Add(new Line(OriginalAxes[i].From, OriginalAxes[i + 1].From));
                JointAngles.Add(0.0);
            }

            Line endLink = new Line(OriginalLinks[OriginalLinks.Count - 1].To, new Vector3d(OriginalLinks[OriginalLinks.Count - 1].To - OriginalLinks[OriginalLinks.Count - 1].From), 30);
            OriginalLinks.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
            RhinoApp.WriteLine("OriginalLinks count = {0}", OriginalLinks.Count);

        }
        public Robot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)        //with planes for applying geometry
        {
            OriginalBodies = bodies;
            OriginalAxes = axes;
            JointRange = jointRange;
            foreach (Line l in OriginalAxes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            for (int i = 0; i < OriginalAxes.Count - 1; i++)
                OriginalLinks.Add(new Line(OriginalAxes[i].From, OriginalAxes[i + 1].From));

            Line endLink = new Line(OriginalLinks[OriginalLinks.Count - 1].To, new Vector3d(OriginalLinks[OriginalLinks.Count - 1].To - OriginalLinks[OriginalLinks.Count - 1].From), 30);
            OriginalLinks.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
            RhinoApp.WriteLine("OriginalLinks count = {0}", OriginalLinks.Count);


        }

        public void InitializeBot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)
        {
            OriginalBodies = bodies;
            OriginalAxes = axes;
            JointRange = jointRange;
            foreach (Line l in OriginalAxes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            for (int i = 0; i < OriginalAxes.Count - 1; i++)
                OriginalLinks.Add(new Line(OriginalAxes[i].From, OriginalAxes[i + 1].From));

            Line endLink = new Line(OriginalLinks[OriginalLinks.Count - 1].To, new Vector3d(OriginalLinks[OriginalLinks.Count - 1].To - OriginalLinks[OriginalLinks.Count - 1].From), 30);
            OriginalLinks.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
            IsSet = true;
            RhinoApp.WriteLine("initializing");
        }
        public void Clear()
        {
            Axes.Clear();
            jointRange.Clear();
            OriginalAxisPlanes.Clear();
            AxisPlanes.Clear();
            OriginalBodies.Clear();
            Bodies.Clear();
            OriginalLinks.Clear();
            Links.Clear();
            JointAngles.Clear();
            Iterations = 0;
            IsSet = false;
        }
        public bool Solve_IK(Point3d target, double threshhold = 0.1, int maxIterations = 5000)            // Location Only
        {
            bool success = false;
            TargetPoint = target;
            threshhold = Math.Pow(threshhold, 2);
            for (int i = Iterations; i < maxIterations; i++)
            {
                stepCCD();
                Iterations = i;
                if (DistError < threshhold)
                {
                    success = true;
                    break;
                }
            }
            RhinoApp.WriteLine("step {0} => case {1}", Iterations, Iterations % 5);

            return success;
        }
        public bool Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)            //  with orientation
        {
            bool success = false;
            TargetPlane = target;
            TargetFrame = target;
            distThreshhold = Math.Pow(distThreshhold, 2);
            for (int i = Iterations; i < maxIterations; i++)
            {

                stepCCD_Orient(i);
                Iterations = i;
                if (DistError < distThreshhold && AngleError < angleThreshhold)
                {
                    success = true;
                    break;
                }
            }
            RhinoApp.WriteLine("step {0} => case {1}", Iterations, Iterations % 5);

            return success;
        }
        public bool Solve_IK(Plane target, DataTree<Line> structure, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)            // on Line Only
        {
            bool successOut = false;
            TargetPlane = target;
            TargetFrame = target;
            double oldDist = 1000000.0;
            distThreshhold = Math.Pow(distThreshhold, 2);

            if (Iterations == 0)
            {
                for (int j = 0; j < 5; j++)
                    stepCCD();
            }
            robotState beforeState = new robotState();
            foreach (double d in JointAngles) beforeState.JointAnglesTemp.Add(d);
            TargetFrame = new Plane(structure.Branch(0)[0].ClosestPoint(EndFrame.Origin, true), structure.Branch(0)[0].Direction, structure.Branch(0)[1].Direction);
            List<robotState> states = new List<robotState>();
            for (int n = 0; n < 4; n++)
            {
                bool success = false;
                TargetFrame.Rotate(Math.PI / 2, structure.Branch(0)[0].Direction, structure.Branch(0)[0].From);
                for (int i = Iterations; i < maxIterations; i++)
                {
                    if (i % 5 == 3) targetFrame.Origin = structure.Branch(0)[0].ClosestPoint(EndFrame.Origin, true);

                    stepCCD_Orient(i);
                    Iterations = i;
                    if (DistError < distThreshhold && AngleError < angleThreshhold)
                    {

                        success = true;
                        break;
                    }
                }
                if (success)
                {
                    robotState stateTemp = new robotState();
                    foreach (double d in JointAngles) stateTemp.JointAnglesTemp.Add(d);
                    stateTemp.TargetDistanceTemp = EndFrame.Origin.DistanceTo(TargetPlane.Origin);
                    states.Add(stateTemp);
                }
                successOut |= success;
            }
            states = states.OrderBy(z => z.TargetDistanceTemp).ToList();
            JointAngles.Clear();
            foreach (double d in states[0].JointAnglesTemp) JointAngles.Add(d);
            for (int i = 0; i <= Axes.Count - 1; i++)           //for each axis-----ORIGIN  
            {
                var rot = Transform.Rotation(JointAngles[i], Axes[i].Direction, Axes[i].From);
                rotateAxes(i, rot);
            }

            //RhinoApp.WriteLine("step {0} => case {1}", Iterations, Iterations % 5);
            return successOut;
        }

        private void stepCCD()     //Location Only
        {
            if (DistError > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                {
                    var rot = Rotate(i, TargetFrame.Origin, EndPoint);
                    rotateAxes(i, rot);
                }
            }
        }

        private void stepCCD_Orient(int iterationCount)
        {
            switch (iterationCount % 5)
            {
                case 0:
                    if (Vector3d.VectorAngle(TargetFrame.ZAxis, EndFrame.ZAxis) > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----X
                        {
                            var rot = Rotate(i, TargetFrame.ZAxis, EndFrame.ZAxis);
                            RhinoApp.WriteLine("WTF OriginalLinks count = {0}", OriginalLinks.Count);

                            rotateAxes(i, rot);
                            RhinoApp.WriteLine("WTF 0");

                        }

                    }
                    break;
                case 1:
                    if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
                    {
                        RhinoApp.WriteLine("WTF 1");

                        for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----X
                        {
                            var rot = Rotate(i, TargetFrame.XAxis, EndFrame.XAxis);
                            rotateAxes(i, rot);
                        }
                    }
                    break;
                case 6:
                    if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----Y
                        {
                            var rot = Rotate(i, TargetFrame.YAxis, EndFrame.YAxis);
                            rotateAxes(i, rot);
                        }
                    }
                    break;
                case 3:
                    if (DistError > 0.01)
                    {
                        RhinoApp.WriteLine("WTF 3");
                        for (int i = 0; i <= Axes.Count - 1; i++)           //for each axis-----ORIGIN  
                        {
                            var rot = Rotate(i, TargetFrame.Origin, EndPoint);
                            rotateAxes(i, rot);
                        }
                    }

                    break;
                case 4:
                    if (DistError > 0.01)
                    {
                        RhinoApp.WriteLine("WTF 4");
                        for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                        {
                            var rot = Rotate(i, TargetFrame.Origin, EndPoint);
                            rotateAxes(i, rot);
                        }
                    }
                    break;
            }
        }

        private Transform Rotate(int i, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - Axes[i].From);
            Vector3d jointToGoal = new Vector3d(target - Axes[i].From);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);
            //RhinoApp.WriteLine("angle[{1}] = {0}", angle, i);

            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            JointAngles[i] += angle;

            var rot = Transform.Rotation(JointAngles[i], Axes[i].Direction, Axes[i].From);
            //RhinoApp.WriteLine("fixed angle[{1}] = {0}", angle, i);
            return rot;
        }

        private Transform Rotate(int i, Vector3d target, Vector3d end)
        {
            double angle = Vector3d.VectorAngle(end, target, AxisPlanes[i]);
            //RhinoApp.WriteLine("angle[{1}] = {0}", angle, i);

            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            JointAngles[i] += angle;
            var rot = Transform.Rotation(JointAngles[i], Axes[i].Direction, Axes[i].From);
            //RhinoApp.WriteLine("fixed angle[{1}] = {0}", angle, i);
            return rot;
        }

        private void rotateAxes(int j, Transform rot)
        {
            Plane endFrameTemp = EndFrame;
            endFrameTemp.Transform(rot);
            EndFrame = endFrameTemp;
            RhinoApp.WriteLine("WTF MATE");

            for (int i = j; i < Axes.Count; i++)        //move link and children
            {
                RhinoApp.WriteLine("AXES count = {0}", Axes.Count);
                RhinoApp.WriteLine("Planes count = {0}", OriginalAxisPlanes.Count);
                RhinoApp.WriteLine("OriginalLinks count = {0}", OriginalLinks.Count);
                RhinoApp.WriteLine("Links count = {0}", Links.Count);

                Line axis = OriginalAxes[i];

                Line link = OriginalLinks[i];
                Plane plane = OriginalAxisPlanes[i];
                axis.Transform(rot);
                link.Transform(rot);
                plane.Transform(rot);

                Axes[i] = axis;
                RhinoApp.WriteLine("WTF FU count = {0}", i);

                Links[i] = link;
                AxisPlanes[i] = plane;

            }
        }

        public void ApplyBodies()
        {
            Bodies.Clear();
            for (int i = 0; i < AxisPlanes.Count; i++)
            {
                var reorient = Transform.PlaneToPlane(OriginalAxisPlanes[i], AxisPlanes[i]);
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



    // </Custom additional code>
}
