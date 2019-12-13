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


    private void RunScript(bool reset, List<Line> axes, List<Line> links, Plane rootFrame, Plane endFrame, Plane target, bool orient, DataTree<Brep> bodies, DataTree<double> range, DataTree<Line> structure, int iterations, double error, double angleError, ref object Axes, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object Solved)
    {
        // <Custom code>

        bool solvedTemp = false;
        if (reset)
        {
            bot.Clear();
            Print("reset");
            RhinoApp.WriteLine("LinksCount = {0}", bot.Links.Count);

        }
        else
        {
            if (!bot.IsSet)
            {
                bot.InitializeBot(axes, links, rootFrame, endFrame, bodies, range);
            }
            if (error == 0) error = 0.1;
            if (angleError == 0) angleError = 0.01;

            if (structure.DataCount == 0)
            {
                if (orient)
                {
                    solvedTemp = bot.Solve_IK(target, error, angleError, iterations);
                    RhinoApp.WriteLine("Solving with orient");

                }
                else
                {
                    solvedTemp = bot.Solve_IK(1, iterations);
                    RhinoApp.WriteLine("Solving no orient");

                }
            }
            else
            {
                if (orient)
                {
                    solvedTemp = bot.Solve_IK(target, structure, error, angleError, iterations);
                    RhinoApp.WriteLine("Solving Line");
                }
            }


            bot.ApplyBodies();
            if (solvedTemp) Print("solve OK");
            Print("iterations = {0}", bot.Iterations);
            Print("ErrorSq = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);
        }
        //bot.Iterations = 0;
        Axes = bot.AxisPlanes;
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
        private List<Line> errorLines = new List<Line> { new Line(), new Line(), new Line() };
        //private List<RobotState> robotStates = new List<RobotState>();
        //private RobotState currentState = new RobotState();
        //private double distError;        //actually square of dist to shorten calculation time
        //private double angleError;
        //private Point3d endPoint;
        private Plane originalRootFrame;
        private Plane rootFrame;
        private Plane originalEndFrame;
        private Plane endFrame;
        private Point3d targetPoint;
        private Plane targetFrame;
        private int iterations;
        private bool isSet = false;

        public List<Line> OriginalAxes { get { return axes; } set { axes = value; } }
        public List<Line> Axes { get { return axes; } set { axes = value; } }
        public DataTree<double> JointRange
        { get { return jointRange; } set { jointRange = value; } }
        public List<Plane> OriginalAxisPlanes
        { get { return originalAxisPlanes; } set { originalAxisPlanes = value; } }
        public List<Plane> AxisPlanes
        { get { return axisPlanes; } set { axisPlanes = value; } }
        public DataTree<Brep> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Brep> Bodies { get { return bodies; } set { bodies = value; } }
        public List<Line> OriginalLinks { get { return links; } set { links = value; } }
        public List<Line> Links { get { return links; } set { links = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public List<Line> ErrorLines { get { return errorLines; } set { errorLines = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public Plane OriginalRootFrame { get { return endFrame; } set { endFrame = value; } }
        public Plane RootFrame { get { return rootFrame; } set { rootFrame = value; } }
        public Plane OriginalEndFrame { get { return originalEndFrame; } set { originalEndFrame = value; } }
        public Plane EndFrame { get { return endFrame; } set { endFrame = value; } }
        public Point3d EndPoint { get { return endFrame.Origin; } }

        public Point3d TargetPoint { get { return targetPoint; } set { targetPoint = value; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        //public List<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }
        //public RobotState CurrentState { get { return currentState; } set { currentState = value; } }

        public Robot() { }
        public Robot(List<Line> axes, DataTree<double> jointRange)           //bare skeleton
        {
            OriginalAxes = axes;
            JointRange = jointRange;

            for (int i = 0; i < Axes.Count - 1; i++)
            {
                OriginalLinks.Add(new Line(Axes[i].From, Axes[i + 1].From));
                JointAngles.Add(0.0);
            }

            Line endLink = new Line(OriginalLinks[OriginalLinks.Count - 1].To, new Vector3d(OriginalLinks[OriginalLinks.Count - 1].To - OriginalLinks[OriginalLinks.Count - 1].From), 30);
            OriginalLinks.Add(endLink);
            OriginalEndFrame = new Plane(endLink.To, endLink.Direction);

            foreach (Line l in OriginalLinks) Links.Add(l);
        }

        public Robot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)        //with planes for applying geometry
        {
            OriginalBodies = bodies;
            OriginalAxes = axes;
            Axes = axes;
            JointRange = jointRange;
            foreach (Line l in Axes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            for (int i = 0; i < Axes.Count - 1; i++)
                OriginalLinks.Add(new Line(Axes[i].From, Axes[i + 1].From));

            Line endLink = new Line(OriginalLinks[OriginalLinks.Count - 1].To, new Vector3d(OriginalLinks[OriginalLinks.Count - 1].To - OriginalLinks[OriginalLinks.Count - 1].From), 30);
            OriginalLinks.Add(endLink);
            OriginalEndFrame = new Plane(endLink.To, endLink.Direction);
            foreach (Line l in OriginalLinks) Links.Add(l);
        }

        //public class RobotState : Robot
        //{
        //    public RobotState() { }
        //}
        public void InitializeBot(List<Line> axes, List<Line> links, Plane rootPlane, Plane endPlane, DataTree<Brep> bodies, DataTree<double> jointRange)
        {
            OriginalAxes = axes;
            OriginalLinks = links;
            OriginalRootFrame = rootPlane;
            OriginalEndFrame = endPlane;
            OriginalBodies = bodies;
            JointRange = jointRange;
            RootFrame = rootPlane;
            EndFrame = endPlane;

            foreach (Line l in axes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
                //CurrentState.JointAngles.Add(0.0);
            }

            IsSet = true;
            RhinoApp.WriteLine("initializing robot");
        }
        public void Clear()
        {
            Axes = OriginalAxes;
            Bodies = OriginalBodies;
            Links = OriginalLinks;
            AxisPlanes.Clear();
            OriginalAxisPlanes.Clear();
            JointAngles.Clear();
            ErrorLines.Clear();
            Iterations = 0;
            IsSet = false;
        }
        public bool Solve_IK(double threshhold = 0.1, int maxIterations = 5000)            // Location Only
        {
            bool success = false;
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
            return success;
        }
        public bool Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)            //  with orientation
        {
            bool success = false;
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
            bool success = false;
            //for (int n = 0; n < 4; n++)
            {
                RhinoApp.WriteLine("with structure");
                TargetFrame = target;
                distThreshhold = Math.Pow(distThreshhold, 2);
                if (Iterations == 0)
                {
                    for (int j = 0; j < 5; j++)
                    {
                        if (Iterations < maxIterations)
                        {
                            stepCCD();
                            Iterations++;
                        }
                    }
                }
                double squareAngle = Vector3d.VectorAngle(new Vector3d(structure.Branch(0)[0].ClosestPoint(TargetFrame.Origin, false)), structure.Branch(0)[1].Direction, structure.Branch(0)[0].Direction);
                RhinoApp.WriteLine("angle = {0}", squareAngle);
                TargetFrame = new Plane(structure.Branch(0)[0].ClosestPoint(EndFrame.Origin, true), structure.Branch(0)[0].Direction, structure.Branch(0)[1].Direction);
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
            }
            //RhinoApp.WriteLine("step {0} => case {1}", Iterations, Iterations % 5);
            return success;
        }

        private void stepCCD()     //Location Only
        {
            if (DistError > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                {
                    double angle = getAngle(i, TargetFrame.Origin, EndFrame.Origin);
                    goToAngle(i, angle);
                }
            }
        }

        private void stepCCD_Orient(int iterationCount)
        {
            switch (iterationCount % 5)
            {
                case 0:           //for each axis-----Z
                    if (Vector3d.VectorAngle(TargetFrame.ZAxis, EndFrame.ZAxis) > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, TargetFrame.ZAxis, EndFrame.ZAxis);
                            goToAngle(i, angle);
                        }
                    }
                    break;
                case 1:          //for each axis-----X
                    if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, TargetFrame.XAxis, EndFrame.XAxis);
                            goToAngle(i, angle);
                        }
                    }
                    break;
                case 6:           //for each axis-----Y
                    if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, TargetFrame.YAxis, EndFrame.YAxis);
                            goToAngle(i, angle);

                        }
                    }
                    break;
                case 3:

                    if (DistError > 0.01)
                    {
                        for (int i = 0; i <= Axes.Count - 1; i++)           //for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, TargetFrame.Origin, EndFrame.Origin);
                            goToAngle(i, angle);
                        }
                    }

                    break;
                case 4:

                    if (DistError > 0.01)
                    {
                        for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, TargetFrame.Origin, EndFrame.Origin);
                            goToAngle(i, angle);
                        }
                    }
                    break;
            }
        }

        private double getAngle(int i, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - Axes[i].From);
            Vector3d jointToGoal = new Vector3d(target - Axes[i].From);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            RhinoApp.WriteLine("joint {0} angle = {1}", i, angle);

            return angle;
        }

        private double getAngle(int i, Vector3d target, Vector3d end)
        {
            double angle = Vector3d.VectorAngle(end, target, AxisPlanes[i]);
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            return angle;
        }




        public void goToAngle(int joint, double angle)
        {
            RhinoApp.WriteLine("angle = {0}", JointAngles[joint]);

            var rot = Transform.Rotation(angle - JointAngles[joint], AxisPlanes[joint].Normal, AxisPlanes[joint].Origin);

            Plane endFrameTemp = EndFrame;
            endFrameTemp.Transform(rot);
            EndFrame = endFrameTemp;

            for (int i = joint; i < Axes.Count; i++)        //move link and children
            {
                RhinoApp.WriteLine("rot");

                Plane plane = AxisPlanes[i];
                plane.Transform(rot);
                AxisPlanes[i] = plane;
            }
            JointAngles[joint] += angle;
        }
        //public void goToState(RobotState state)
        //{
        //    for (int j = 0; j < JointAngles.Count; j++)
        //    {
        //        var rot = Transform.Rotation(state.JointAngles[j], AxisPlanes[j].Normal, AxisPlanes[j].Origin);

        //        Plane endFrameTemp = OriginalEndFrame;
        //        endFrameTemp.Transform(rot);
        //        state.EndFrame = endFrameTemp;

        //        for (int i = j; i < Axes.Count; i++)        //move link and children
        //        {
        //            Plane plane = OriginalAxisPlanes[i];
        //            plane.Transform(rot);
        //            state.AxisPlanes[i] = plane;
        //        }
        //    }
        //}
        public void ApplyBodies()
        {
            Bodies.Clear();
            for (int i = 0; i < AxisPlanes.Count; i++)
            {
                var reorient = Transform.PlaneToPlane(OriginalAxisPlanes[i], AxisPlanes[i]);
                foreach (Brep b in OriginalBodies.Branch(new GH_Path(i + 1)))
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
