using System;
using System.Collections.Generic;
using System.IO;

public class Rover
{
    //Heuristic function for eight-direction movement
    public static int Function(VertexPlace a, VertexPlace b)
    {
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    public static void CalculateRoverPath(string[,] map)
    {
        try
        {
            var graph = new LatticeGraph(MatrixFromStrMatrix(map));
            var astar = new AStarSearch(graph);

            var path = astar.ExtractPath();
            path.Reverse();
            var fuel = astar.ExtractCost(path);
            var steps = path.Count - 1;

            var pathString = $"[{path[0].X}][{path[0].Y}]";
            if (steps > 0)
                for (var i = 1; i <= steps; i++)
                    pathString += $"->[{path[i].X}][{path[i].Y}]";

            var stringToFile = pathString + "\n" + $"steps: {steps}\n" + $"fuel: {fuel}";
            FileHelper.WriteInFile(stringToFile);
        }
        catch (CannotStartMovement ex)
        {
            FileHelper.WriteInFile(ex.Message);
        }
        catch (Exception ex)
        {
            FileHelper.WriteInFile(ex.Message);
        }
    }

    private static int?[,] MatrixFromStrMatrix(string[,] array)
    {
        if (array[0, 0] == "X") throw new CannotStartMovement("First element is unreachable");

        if (array[array.GetLength(0) - 1, array.GetLength(1) - 1] == "X")
            throw new CannotStartMovement("Last element is unreachable");

        var output = new int?[array.GetLength(0), array.GetLength(1)];
        for (var i = 0; i < array.GetLength(0); i++)
        for (var j = 0; j < array.GetLength(1); j++)
        {
            if (!int.TryParse(array[i, j], out _) && !array[i, j].Equals("X"))
                throw new CannotStartMovement("Wrong matix");

            if (array[i, j].Equals("X"))
                output[i, j] = null;
            else
                output[i, j] = int.Parse(array[i, j]);
        }

        return output;
    }

    public static bool isDiagonalStep(VertexPlace a, VertexPlace b)
    {
        if (Math.Abs(a.X - b.X) == 1 && Math.Abs(a.Y - b.Y) == 1) return true;

        return false;
    }
}

public class CannotStartMovement : Exception
{
    public CannotStartMovement(string message)
        : base(message)
    {
    }

    public CannotStartMovement(string message, Exception inner)
        : base(message, inner)
    {
    }
}

public static class FileHelper
{
    public static void WriteInFile(string text)
    {
        var fileName = "path-plan.txt";
        File.WriteAllTextAsync(fileName, text);
    }
}

public class LatticeGraph
{
    public int?[,] weights;

    public int width;
    public int height;

    private readonly int[][] directions =
    {
        //Right
        new[] {1, 0},
        //Left
        new[] {-1, 0},
        //Down
        new[] {0, 1},
        //Up
        new[] {0, -1},
        //Bottom right
        new[] {1, 1},
        //Bottom left 
        new[] {-1, 1},
        //Top right
        new[] {1, -1},
        //Top left
        new[] {-1, -1}
    };

    public LatticeGraph(int?[,] array)
    {
        weights = array;
        width = array.GetLength(0);
        height = array.GetLength(1);
    }

    //returns: weight of edge between two neighbour vertexes in graph
    public int? GetCost(VertexPlace a, VertexPlace b)
    {
        var x1 = a.X;
        var y1 = a.Y;
        var x2 = b.X;
        var y2 = b.Y;
        if (weights[x1, y1] != null && weights[x2, y2] != null)
            return Math.Abs((int) weights[x1, y1] - (int) weights[x2, y2]) + 1;

        return null;
    }


    private bool IsInMatrix(VertexPlace coords)
    {
        return 0 <= coords.X && coords.X < width && 0 <= coords.Y && coords.Y < height;
    }

    public IEnumerable<VertexPlace> GetNeighbours(VertexPlace coords)
    {
        var listOfNeighbours = new List<VertexPlace>();
        foreach (var dir in directions)
        {
            var neighbour = new VertexPlace(coords.X + dir[0], coords.Y + dir[1]);
            if (IsInMatrix(neighbour))
                if (weights[neighbour.X, neighbour.Y] != null)
                    listOfNeighbours.Add(neighbour);
        }

        return listOfNeighbours;
    }
}

public class VertexPlace
{
    public int X;
    public int Y;

    public VertexPlace(int x, int y)
    {
        X = x;
        Y = y;
    }

    public override bool Equals(object? obj)
    {
        return Equals(obj as VertexPlace);
    }

    protected bool Equals(VertexPlace other)
    {
        return X == other.X && Y == other.Y;
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(X, Y);
    }
}

public class Vertex
{
    public VertexPlace From;
    public VertexPlace To;
}

public class QueueWithPriority
{
    private readonly List<Tuple<VertexPlace, int?>> elements = new();

    public int Count => elements.Count;

    public void Enqueue(VertexPlace item, int? priority)
    {
        elements.Add(Tuple.Create(item, priority));
    }

    public VertexPlace Dequeue()
    {
        var bestIndex = 0;

        for (var i = 0; i < elements.Count; i++)
            if (elements[i].Item2 < elements[bestIndex].Item2)
                bestIndex = i;

        var bestItem = elements[bestIndex].Item1;
        elements.RemoveAt(bestIndex);
        return bestItem;
    }
}

public class AStarSearch
{
    private readonly List<Vertex> pathVertices = new();
    private readonly Dictionary<VertexPlace, int?> verticesCost = new();
    public LatticeGraph graph;

    public AStarSearch(LatticeGraph g)
    {
        graph = g;
        var start = new VertexPlace(0, 0);
        var goal = new VertexPlace(graph.width - 1, graph.height - 1);
        var reachableVertices = new QueueWithPriority();
        reachableVertices.Enqueue(start, 0);

        pathVertices.Add(new Vertex {From = null, To = start});
        verticesCost[start] = 0;
        var diag = 1;
        while (reachableVertices.Count > 0)
        {
            var current = reachableVertices.Dequeue();

            if (current.X == goal.X && current.Y == goal.Y) break;

            var isIncremented = false;
            foreach (var next in graph.GetNeighbours(current))
            {
                var newCost = verticesCost[current] + graph.GetCost(current, next);
                if (!verticesCost.ContainsKey(next) || newCost < verticesCost[next])
                {
                    if (Rover.isDiagonalStep(current, next))
                    {
                        if (diag % 2 == 0) newCost += 1;

                        if (!isIncremented) diag++;

                        isIncremented = true;
                    }

                    verticesCost[next] = newCost;
                    var priority = newCost + Rover.Function(next, goal);
                    reachableVertices.Enqueue(next, priority);
                    pathVertices.Add(new Vertex {From = current, To = next});
                }
            }
        }
    }

    public List<VertexPlace> ExtractPath()
    {
        if (pathVertices.Count == 1) return new List<VertexPlace> {new(pathVertices[0].To.X, pathVertices[0].To.Y)};

        var path = new List<VertexPlace>();
        var elem = pathVertices.Find(x => Equals(x.To, new VertexPlace(graph.width - 1, graph.height - 1)));
        path.Add(elem.To);
        var fromElem = elem.From;
        while (fromElem != null && !fromElem.Equals(new VertexPlace(0, 0)))
        {
            path.Add(fromElem);
            fromElem = pathVertices.Find(x => Equals(x.To, fromElem))?.From;
        }

        path.Add(pathVertices[0].To);

        return path;
    }

    public int ExtractCost(List<VertexPlace> path)
    {
        if (path.Count == 1) return 0;

        var len = path.Count;
        var sum = 0;
        var diag = 1;
        for (var i = 1; i < len; i++)
        {
            if (Rover.isDiagonalStep(path[i], path[i - 1]))
            {
                if (diag % 2 == 0) sum += 1;

                diag++;
            }

            sum += (int) graph.GetCost(path[i], path[i - 1]);
        }

        return sum;
    }
}