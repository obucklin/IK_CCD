﻿using System;
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


    private void RunScript(bool reset, List<Line> x, Plane t, bool or, DataTree<Brep> g, DataTree<double> r, int i, double ed, double ea, ref object A, ref object B, ref object C)
    {
        // <Custom code>
        if (reset)
        {
            bot.Clear();
            Print("reset");
        }
        else
        {
            if (!bot.IsSet)
            {
                bot.InitializeBot(x, g, r);
            }
            if (ed == 0) ed = 0.1;
            if (ea == 0) ea = 0.01;

            if (or) bot.Solve_IK(t, ed, ea, i);
            else bot.Solve_IK(t.Origin, 1, i);
            Print("solve OK");
            bot.ApplyBodies();
            Print("iterations = {0}", bot.Iterations);
            Print("ErrorSq = {0}", bot.DistError);
        }
        A = bot.Links;
        B = bot.EndFrame;
        C = bot.Bodies;

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
        private List<Line> links = new List<Line>();
        private List<double> jointAngles = new List<double>();
        private List<Line> errorLines = new List<Line> { new Line(), new Line(), new Line() };
        private double distError;        //actually square of dist to shorten calculation time
        private double angleError;
        private Point3d endPoint;
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
        public List<Line> Links { get { return links; } set { links = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public List<Line> ErrorLines { get { return errorLines; } set { errorLines = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public Point3d EndPoint { get { return endFrame.Origin; } }
        public Plane EndFrame { get { return endFrame; } set { endFrame = value; } }
        public Point3d TargetPoint { get { return targetPoint; } set { targetPoint = value; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }

        public Robot() { }

        public Robot(List<Line> axes, DataTree<double> jointRange)           //bare skeleton
        {
            Axes = axes;
            JointRange = jointRange;

            for (int i = 0; i < Axes.Count - 1; i++)
            {
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));
                JointAngles.Add(0.0);
            }
            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
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
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));

            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);

        }

        public void InitializeBot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)
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
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));

            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
            IsSet = true;
            RhinoApp.WriteLine("init");
        }


        public void Clear()
        {
            Axes.Clear();
            jointRange.Clear();
            OriginalAxisPlanes.Clear();
            AxisPlanes.Clear();
            OriginalBodies.Clear();
            Bodies.Clear();
            Links.Clear();
            JointAngles.Clear();
            ErrorLines.Clear();
            Iterations = 0;
            IsSet = false;
        }

        private void stepCCD()     //Location Only
        {
            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis
            {
                Vector3d jointToEnd = new Vector3d(Links[Links.Count - 1].To - Axes[i].From);
                Vector3d jointToGoal = new Vector3d(TargetPoint - Axes[i].From);

                double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);

                if (angle > Math.PI) angle = angle - (2 * Math.PI);
                if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                    angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
                var rot = Transform.Rotation(angle, Axes[i].Direction, Axes[i].From);
                JointAngles[i] += angle;
                EndFrame.Transform(rot);

                for (int j = i; j < Axes.Count; j++)        //move link and children
                {
                    Line axis = Axes[j];
                    Line link = Links[j];
                    Plane plane = AxisPlanes[j];
                    axis.Transform(rot);
                    link.Transform(rot);
                    plane.Transform(rot);
                    Axes[j] = axis;
                    Links[j] = link;
                    AxisPlanes[j] = plane;
                }
            }
        }

        public void Solve_IK(Point3d target, double threshhold = 0.1, int iterations = 5000)            // Location Only
        {
            TargetPoint = target;
            threshhold = Math.Pow(threshhold, 2);
            for (int i = Iterations; i < iterations; i++)
            {
                stepCCD();
                if (DistError < threshhold)
                {
                    Iterations = i;
                    break;
                }
            }
        }

        public void Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int iterations = 5000)            // Location Only
        {
            TargetFrame = target;
            distThreshhold = Math.Pow(distThreshhold, 2);
            for (int i = Iterations; i < iterations; i++)
            {
                stepCCD_Orient();
                Iterations = i;
                if (DistError < distThreshhold && AngleError < angleThreshhold)
                    break;
            }
        }

        private void stepCCD_Orient()
        {
            if (Vector3d.VectorAngle(TargetFrame.ZAxis, EndFrame.ZAxis) > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----X
                {
                    Point3d targetZ = TargetFrame.Origin + TargetFrame.ZAxis * 100;
                    Point3d eeZ = EndPoint + EndFrame.ZAxis * 100;
                    var rot = Rotate(i, targetZ, eeZ);
                    rotateAxes(i, rot);
                }
            }
            if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----X
                {
                    Point3d targetX = TargetFrame.Origin + TargetFrame.XAxis * 100;
                    Point3d eeX = EndPoint + EndFrame.XAxis * 100;
                    var rot = Rotate(i, targetX, eeX);
                    rotateAxes(i, rot);
                }
            }
            if (Vector3d.VectorAngle(TargetFrame.XAxis, EndFrame.XAxis) > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----Y
                {
                    Point3d targetY = TargetFrame.Origin + TargetFrame.YAxis * 100;
                    Point3d eeY = EndPoint + EndFrame.YAxis * 100;
                    var rot = Rotate(i, targetY, eeY);
                    rotateAxes(i, rot);

                }
            }

            if (DistError > 0.01)
            {
                for (int i = 0; i <= Axes.Count - 1; i++)           //for each axis-----ORIGIN  
                {
                    var rot = Rotate(i, TargetFrame.Origin, EndPoint);
                    rotateAxes(i, rot);
                }
            }

            if (DistError > 0.01)
            {
                for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                {
                    var rot = Rotate(i, TargetFrame.Origin, EndPoint);
                    rotateAxes(i, rot);
                }
            }
        }

        private Transform Rotate(int i, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - Axes[i].From);
            Vector3d jointToGoal = new Vector3d(target - Axes[i].From);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);
            RhinoApp.WriteLine("angle[{1}] = {0}", angle, i);

            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            var rot = Transform.Rotation(angle, Axes[i].Direction, Axes[i].From);
            RhinoApp.WriteLine("fixed angle[{1}] = {0}", angle, i);
            JointAngles[i] += angle;
            return rot;
        }

        private void rotateAxes(int j, Transform rot)
        {
            Plane endFrameTemp = EndFrame;
            endFrameTemp.Transform(rot);
            EndFrame = endFrameTemp;

            for (int i = j; i < Axes.Count; i++)        //move link and children
            {
                Line axis = Axes[i];
                Line link = Links[i];
                Plane plane = AxisPlanes[i];
                axis.Transform(rot);
                link.Transform(rot);
                plane.Transform(rot);
                Axes[i] = axis;
                Links[i] = link;
                AxisPlanes[i] = plane;
            }
        }

        public void ApplyBodies()
        {
            Bodies.Clear();
            //RhinoApp.WriteLine("AxisPlanes.Count = {0}", AxisPlanes.Count);
            //RhinoApp.WriteLine("originalBodies.Count = {0}", OriginalBodies.BranchCount);
            for (int i = 0; i < AxisPlanes.Count; i++)
            {
                //RhinoApp.WriteLine("iterator = {0}", i);
                var reorient = Transform.PlaneToPlane(OriginalAxisPlanes[i], AxisPlanes[i]);
                foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                {
                    //RhinoApp.WriteLine("bodiesBranch = {0}", i);
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
