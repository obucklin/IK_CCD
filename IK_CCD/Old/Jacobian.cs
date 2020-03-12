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


    
    private void RunScript(bool tan, List<Line> axes, List<double> y, Plane target, int n, ref object A, ref object B, ref object C, ref object D)
    {
        // <Custom code> 


        Plane pl = endPlane();
        List<double> axesOut = new List<double>();
        List<Line> errorLine = new List<Line>();
        List<Line> jakOut = new List<Line>();
        double error = 100;

        for (int i = 0; i < n; i++)
        {
            Plane plOut;
            moveEnd(axes, axes, pl, out plOut);
            pl = plOut;
            jakOut.Clear();
            if (tan) axesOut = axisDeltaTan(axes, jakOut, pl, target, .01, out errorLine, out error);
            else axesOut = axisDelta(axes, jakOut, pl, target, .001, out errorLine, out error);
            if (error < 0.01)
            {
                Print("iterations = {0}", i.ToString());
                break;
            }
        }
        Print("error = {0}", error.ToString());
        A = axes;
        B = pl;
        C = axesOut;
        D = errorLine;




        // </Custom code> 
    }


    // <Custom additional code> 
    public List<double> axisDelta(List<Line> axes, List<Line> jakOut, Plane eePl, Plane target, double step, out List<Line> errorLines, out double error)
    {
        //target plane points
        Point3d targetCP = target.Origin;
        Point3d targetX = targetCP + target.XAxis * 10;
        Point3d targetY = targetCP + target.YAxis * 10;

        //end effector plane pts
        Point3d eeCP = eePl.Origin;
        Point3d eeX = eeCP + eePl.XAxis * 10;
        Point3d eeY = eeCP + eePl.YAxis * 10;

        //error vectors/lines
        Vector3d errorV = new Vector3d(eeCP, targetCP);
        Vector3d errorVX = new Vector3d(eeX, targetX);
        Vector3d errorVY = new Vector3d(eeY, targetY);

        errorLines = new List<Vector3d> { errorV, errorVX, errorVY };
        error = (errorV.Length + errorVX.Length + errorVY.Length) / 3;

        List<double> deltaTemp = new List<double>();

        for (int i = 0; i < axis.Count; i++)
        {
            //vector axis to end effector points
            Vector3d vectorToEnd = eePl.Origin - axes[i].From;
            Vector3d vectorToX = eeX - axes[i].From;
            Vector3d vectorToY = eeY - axes[i].From;

            //angular vector of rotation about axis[i]
            Vector3d jakobianTpCP = Vector3d.CrossProduct(axes[i].Direction, vectorToEnd);
            Vector3d jakobianTpX = Vector3d.CrossProduct(axes[i].Direction, vectorToX);
            Vector3d jakobianTpY = Vector3d.CrossProduct(axes[i].Direction, vectorToY);

            
            
            
            
            jakobianCP.Unitize();
            jakobianX.Unitize();
            jakobianY.Unitize();

            Vector3d unitErrorV = errorV.Unitize();
            Vector3d unitErrorVX = errorVX.Unitize();
            Vector3d unitErrorVY = errorVY.Unitize();

            double deltaAvg = jakobianCP * unitErrorV;
            deltaAvg += jakobianX * unitErrorVX;
            deltaAvg += jakobianY * unitErrorVY;
            deltaAvg *= step;
            deltaTemp.Add(deltaAvg);
        }
        return deltaTemp;
    }
    public List<double> axisDeltaTan(List<Line> axis, List<Line> jakOut, Plane pl, Plane target, double step, out List<Line> errorLines, out double error)
    {
        //target plane points
        Point3d tCP = target.Origin;
        Point3d tX = tCP + target.XAxis * 10;
        Point3d tY = tCP + target.YAxis * 10;

        //end effector plane pts
        Point3d eCP = pl.Origin;
        Point3d eX = eCP + pl.XAxis * 10;
        Point3d eY = eCP + pl.YAxis * 10;

        //error vectors/lines
        Line ev = new Line(eCP, tCP);
        Line evX = new Line(eX, tX);
        Line evY = new Line(eY, tY);

        errorLines = new List<Line> { ev, evX, evY };
        error = (ev.Length + evX.Length + evY.Length) / 3;

        List<double> deltaTemp = new List<double>();

        for (int i = 0; i < axis.Count; i++)
        {
            //vector axis to end effector points
            Vector3d vectorToEnd = pl.Origin - axis[i].From;
            Vector3d vectorToX = eX - axis[i].From;
            Vector3d vectorToY = eY - axis[i].From;

            //angular vector of rotation about axis[i]
            Vector3d jakobianCP = Vector3d.CrossProduct(axis[i].Direction, vectorToEnd);
            Vector3d jakobianX = Vector3d.CrossProduct(axis[i].Direction, vectorToX);
            Vector3d jakobianY = Vector3d.CrossProduct(axis[i].Direction, vectorToY);

            double deltaAvg = Math.Atan(jakobianCP * ev.Direction / vectorToEnd.Length);
            deltaAvg += Math.Atan(jakobianX * evX.Direction / vectorToX.Length);
            deltaAvg += Math.Atan(jakobianY * evY.Direction / vectorToY.Length);
            deltaAvg *= step;
            deltaTemp.Add(deltaAvg);
        }
        return deltaTemp;
    }

    public List<double> axisDeltaTest(List<Line> x, List<Line> jakOut, Plane pl, Plane target, double step, out double error)
    {
        Point3d tCP = target.Origin;
        Point3d tX = tCP + target.XAxis * 10;
        Point3d tY = tCP + target.YAxis * 10;
        Point3d eCP = pl.Origin;
        Point3d eX = eCP + pl.XAxis * 10;
        Point3d eY = eCP + pl.YAxis * 10;

        Vector3d ev = tCP - eCP;
        Vector3d evX = tX - eX;
        Vector3d evY = tY - eY;

        List<double> errors = new List<double> { ev.Length, evX.Length, evY.Length };
        errors.Sort();
        error = errors[0];

        List<double> deltaTemp = new List<double>();

        for (int i = 0; i < x.Count; i++)
        {
            Vector3d vectorToEnd = pl.Origin - x[i].From;
            Vector3d jakobianCP = Vector3d.CrossProduct(x[i].Direction, vectorToEnd);

            Vector3d vectorToX = eX - x[i].From;
            Vector3d jakobianX = Vector3d.CrossProduct(x[i].Direction, vectorToX);

            Vector3d vectorToY = eY - x[i].From;
            Vector3d jakobianY = Vector3d.CrossProduct(x[i].Direction, vectorToY);

            double deltaAvg = jakobianCP * ev * Math.Pow((ev.Length * step), 2);
            deltaAvg += jakobianX * evX * Math.Pow((evX.Length * step), 2);
            deltaAvg += jakobianY * evY * Math.Pow((evY.Length * step), 2);
            deltaTemp.Add(deltaAvg);
        }
        return deltaTemp;
    }
    public List<double> axisDeltaTanTest(List<Line> x, List<Line> jakOut, Plane pl, Plane target, double step, out List<Line> error)
    {
        Point3d tCP = target.Origin;
        Point3d tX = tCP + target.XAxis * 10;
        Point3d tY = tCP + target.YAxis * 10;
        Point3d eCP = pl.Origin;
        Point3d eX = eCP + pl.XAxis * 10;
        Point3d eY = eCP + pl.YAxis * 10;

        Line ev = new Line(eCP, tCP);
        Line evX = new Line(eX, tX);
        Line evY = new Line(eY, tY);

        List<Line> errorsTemp = new List<Line> { ev, evX, evY };
        errorsTemp = errorsTemp.OrderByDescending(l => l.Length).ToList();
        error = errorsTemp;
        double errorAvg = (errorsTemp[0].Length + errorsTemp[1].Length + errorsTemp[2].Length) / 3;

        List<double> deltaTemp = new List<double>();

        for (int i = 0; i < x.Count; i++)
        {
            Vector3d vectorToEnd = pl.Origin - x[i].From;
            Vector3d jakobianCP = Vector3d.CrossProduct(x[i].Direction, vectorToEnd);

            Vector3d vectorToX = eX - x[i].From;
            Vector3d jakobianX = Vector3d.CrossProduct(x[i].Direction, vectorToX);

            Vector3d vectorToY = eY - x[i].From;
            Vector3d jakobianY = Vector3d.CrossProduct(x[i].Direction, vectorToY);

            double deltaAvg = Math.Atan(jakobianCP * ev.Direction / vectorToEnd.Length) * Math.Pow((ev.Length * step), 2);
            deltaAvg += Math.Atan(jakobianX * evX.Direction / vectorToX.Length) * Math.Pow((evX.Length * step), 2);
            deltaAvg += Math.Atan(jakobianY * evY.Direction / vectorToY.Length) * Math.Pow((evY.Length * step), 2);
            deltaTemp.Add(deltaAvg);
        }
        return deltaTemp;
    }

    public void stuff()
    {
        foreach (Layer l in RhinoDoc.ActiveDoc.Layers) Print(l.FullPath);
        RhinoObject axis0 = RhinoDoc.ActiveDoc.Objects.FindByLayer("Axis0")[0];
        RhinoObject axis1 = RhinoDoc.ActiveDoc.Objects.FindByLayer("Axis1")[0];

        List<GeometryBase> axes = new List<GeometryBase>();
        axes.Add(axis0.DuplicateGeometry());
        axes.Add(axis1.DuplicateGeometry());

        List<Point3d> axisPoints = new List<Point3d>();
        List<Vector3d> axisVectors = new List<Vector3d>();

        List<Line> x = new List<Line>();
        foreach (Line l in x)
        {
            axisPoints.Add(l.From);
            axisVectors.Add(l.Direction);
        }

    }
    public void moveEnd(List<Line> x, List<double> y, Plane endEffector, out Plane plOut)
    {
        Plane pl = endEffector;
        for (int i = 0; i < y.Count && i < x.Count; i++)
        {
            Transform tr = Transform.Rotation(y[i], x[i].Direction, x[i].From);
            for (int j = i + 1; j < x.Count; j++)
            {
                Line xTemp = x[j];
                xTemp.Transform(tr);
                x[j] = xTemp;
            }
            pl.Transform(tr);
        }
        plOut = pl;
    }

    public Plane rootPlane()
    {
        int rootXlayer = RhinoDoc.ActiveDoc.Layers.FindByFullPath("robot::rootPlane::xAxis", true);
        RhinoObject rootX = RhinoDoc.ActiveDoc.Objects.FindByLayer(RhinoDoc.ActiveDoc.Layers[rootXlayer])[0];
        int rootYlayer = RhinoDoc.ActiveDoc.Layers.FindByFullPath("robot::rootPlane::yAxis", true);
        RhinoObject rootY = RhinoDoc.ActiveDoc.Objects.FindByLayer(RhinoDoc.ActiveDoc.Layers[rootYlayer])[0];

        GeometryBase rootXgeo = rootX.DuplicateGeometry();
        GeometryBase rootYgeo = rootY.DuplicateGeometry();
        return new Plane((rootXgeo as LineCurve).Line.From, (rootXgeo as LineCurve).Line.To, (rootYgeo as LineCurve).Line.To);
    }

    public Plane endPlane()
    {
        int endXlayer = RhinoDoc.ActiveDoc.Layers.FindByFullPath("robot::endPlane::xAxis", true);
        RhinoObject endX = RhinoDoc.ActiveDoc.Objects.FindByLayer(RhinoDoc.ActiveDoc.Layers[endXlayer])[0];
        int endYlayer = RhinoDoc.ActiveDoc.Layers.FindByFullPath("robot::endPlane::yAxis", true);
        RhinoObject endY = RhinoDoc.ActiveDoc.Objects.FindByLayer(RhinoDoc.ActiveDoc.Layers[endYlayer])[0];

        GeometryBase endXgeo = endX.DuplicateGeometry();
        GeometryBase endYgeo = endY.DuplicateGeometry();
        return new Plane((endXgeo as LineCurve).Line.From, (endXgeo as LineCurve).Line.To, (endYgeo as LineCurve).Line.To);
    }
    // </Custom additional code>  
}

