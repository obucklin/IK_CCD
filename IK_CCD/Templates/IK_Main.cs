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


    private void RunScript(List<Line> x, Point3d t, DataTree<Brep> g, int i, DataTree<double> r, ref object A, ref object B)
    {
        // <Custom code>

        Robot bot = new Robot(x, g, r);
        for (int j = 0; j < i; j++)
        {
            bot.stepCCD(t);
            Line errorLine = new Line(bot.Links[bot.Links.Count - 1].To, t);
            if (errorLine.Length < 0.1)
            {
                Print("iterations = {0}", j);
                break;
            }
        }

        bot.ApplyBodies();

        A = bot.Links;
        B = bot.Bodies;

        // </Custom code>
    }

    // <Custom additional code>

    public class Robot
    {
        public List<Line> Axes = new List<Line>();
        public DataTree<double> range = new DataTree<double>();
        public List<Plane> OriginalAxisPlanes = new List<Plane>();
        public List<Plane> AxisPlanes = new List<Plane>();
        public DataTree<Brep> OriginalBodies = new DataTree<Brep>();
        public DataTree<Brep> Bodies = new DataTree<Brep>();
        public List<Line> Links = new List<Line>();
        public List<double> jointAngles = new List<double>();
        public Point3d End;

        public Robot(List<Line> axes, DataTree<double> jointRange)           //bare skeleton
        {
            Axes = axes;
            range = jointRange;

            for (int i = 0; i < Axes.Count - 1; i++)
            {
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));
                jointAngles.Add(0.0);
            }
            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
        }

        public Robot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)        //with planes for applying geometry
        {
            OriginalBodies = bodies;
            Axes = axes;
            range = jointRange;
            foreach (Line l in Axes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                jointAngles.Add(0.0);
            }
            for (int i = 0; i < Axes.Count - 1; i++)
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));

            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
        }


        public void stepCCD(Point3d target)
        {
            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis
            {
                Vector3d jointToEnd = new Vector3d(Links[Links.Count - 1].To - Axes[i].From);
                Vector3d jointToGoal = new Vector3d(target - Axes[i].From);

                double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);

                if (angle > Math.PI) angle = angle - (2 * Math.PI);
                if (range.Branch(new GH_Path(i))[0] != 0 && (range.Branch(new GH_Path(i))[0] > (jointAngles[i] + angle) || (jointAngles[i] + angle) > range.Branch(new GH_Path(i))[1]))
                    angle = range.Branch(new GH_Path(i))[1] - jointAngles[i];
                var rot = Transform.Rotation(angle, Axes[i].Direction, Axes[i].From);
                jointAngles[i] += angle;

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

        public void ApplyBodies()
        {
            for (int i = 0; i < AxisPlanes.Count; i++)
            {
                var reorient = Transform.PlaneToPlane(OriginalAxisPlanes[i], AxisPlanes[i]);
                foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                {
                    Brep brepTemp = new Brep();
                    brepTemp = b;
                    brepTemp.Transform(reorient);
                    Bodies.Add(brepTemp, new GH_Path(i));
                }
            }
        }

    }


    // </Custom additional code>
}
