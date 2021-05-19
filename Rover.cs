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

    public static void CalculateRoverPath(int[,] map)
    {
        var graph = new LatticeGraph(map);
        var astar = new AStarSearch(graph);

        var path = astar.ExtractPath();
        path.Reverse();
        var fuel = astar.ExtractCost(path);
        var steps = path.Count - 1;

        var pathString = $"[{path[0].X}][{path[0].Y}]";
        if (steps > 0)
        {
            for (var i = 1; i <= steps; i++)
            {
                pathString += $"->[{path[i].X}][{path[i].Y}]";
            }
        }

        var stringToFile = pathString + "\n" + $"steps: {steps}\n" + $"fuel: {fuel}";
        FileHelper.WriteInFile(stringToFile);
    }
}

public static class FileHelper
{
    public static void WriteInFile(string text)
    {
        var fileName = "path-plan.txt";
        File.WriteAllTextAsync(fileName, text);
    }
    
    public static bool isDiagonalStep(VertexPlace a, VertexPlace b)
    {
        if (Math.Abs(a.X - b.X) == 1 && Math.Abs(a.Y - b.Y) == 1)
        {
            return true;
        }

        return false;
    }
}

public class LatticeGraph
{
    public int[,] weights;

    public int width;
    public int height;

    private int[][] directions =
    {
        //Right
        new int[] {1, 0},
        //Left
        new int[] {-1, 0},
        //Down
        new int[] {0, 1},
        //Up
        new int[] {0, -1},
        //Bottom right
        new int[] {1, 1},
        //Bottom left 
        new int[] {-1, 1},
        //Top right
        new int[] {1, -1},
        //Top left
        new int[] {-1, -1}
    };

    public LatticeGraph(int[,] array)
    {
        weights = array;
        width = array.GetLength(0);
        height = array.GetLength(1);
    }

    //returns: weight of edge between two neighbour vertexes in graph
    public int GetCost(VertexPlace a, VertexPlace b)
    {
        var x1 = a.X;
        var y1 = a.Y;
        var x2 = b.X;
        var y2 = b.Y;
        return Math.Abs(weights[x1, y1] - weights[x2, y2]) + 1;
    }

    

    private bool IsInMatrix(VertexPlace coords)
    {
        return (0 <= coords.X && coords.X < width && 0 <= coords.Y && coords.Y < height);
    }

    public IEnumerable<VertexPlace> GetNeighbours(VertexPlace coords)
    {
        var listOfNeighbours = new List<VertexPlace>();
        foreach (var dir in directions)
        {
            var neighbour = new VertexPlace(coords.X + dir[0], coords.Y + dir[1]);
            if (IsInMatrix(neighbour))
            {
                listOfNeighbours.Add(neighbour);
            }
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
        this.X = x;
        this.Y = y;
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
    private List<Tuple<VertexPlace, double>> elements = new();

    public int Count => elements.Count;

    public void Enqueue(VertexPlace item, double priority)
    {
        elements.Add(Tuple.Create(item, priority));
    }

    public VertexPlace Dequeue()
    {
        int bestIndex = 0;

        for (int i = 0; i < elements.Count; i++)
        {
            if (elements[i].Item2 < elements[bestIndex].Item2)
            {
                bestIndex = i;
            }
        }

        var bestItem = elements[bestIndex].Item1;
        elements.RemoveAt(bestIndex);
        return bestItem;
    }
}

public class AStarSearch
{
    private List<Vertex> pathVertices = new();
    private Dictionary<VertexPlace, int> verticesCost = new();
    public LatticeGraph graph;

    public AStarSearch(LatticeGraph g)
    {
        graph = g;
        var start = new VertexPlace(0, 0);
        var goal = new VertexPlace(graph.width - 1, graph.height - 1);
        var reachableVertices = new QueueWithPriority();
        reachableVertices.Enqueue(start, 0);

        pathVertices.Add(new Vertex() {From = null, To = start});
        verticesCost[start] = 0;
        var diag = 1;
        while (reachableVertices.Count > 0)
        {
            var current = reachableVertices.Dequeue();

            if (current.X == goal.X && current.Y == goal.Y)
            {
                break;
            }

            var isIncremented = false;
            foreach (var next in graph.GetNeighbours(current))
            {
                var newCost = verticesCost[current] + graph.GetCost(current, next);
                if (!verticesCost.ContainsKey(next) || newCost < verticesCost[next])
                {
                    if (FileHelper.isDiagonalStep(current, next))
                    {
                        if (diag % 2 == 0)
                        {
                            newCost += 1;
                        }

                        if (!isIncremented)
                        {
                            diag++;
                        }

                        isIncremented = true;
                    }

                    verticesCost[next] = newCost;
                    var priority = newCost + Rover.Function(next, goal);
                    reachableVertices.Enqueue(next, priority);
                    pathVertices.Add(new Vertex() {From = current, To = next});
                }
            }
        }
    }

    public List<VertexPlace> ExtractPath()
    {
        if (pathVertices.Count == 1)
        {
            return new List<VertexPlace>() {new VertexPlace(pathVertices[0].To.X, pathVertices[0].To.Y)};
        }

        var path = new List<VertexPlace>();
        var elem = pathVertices.Find(x => Equals(x.To, new VertexPlace(graph.width - 1, graph.height - 1)));
        path.Add(elem.To);
        var fromelem = elem.From;
        while (fromelem != null && !fromelem.Equals(new VertexPlace(0, 0)))
        {
            path.Add(fromelem);
            fromelem = pathVertices.Find(x => Equals(x.To, fromelem))?.From;
        }

        path.Add(pathVertices[0].To);

        return path;
    }

    public int ExtractCost(List<VertexPlace> path)
    {
        if (path.Count == 1)
        {
            return 0;
        }

        var len = path.Count;
        var sum = 0;
        var diag = 1;
        for (int i = 1; i < len; i++)
        {
            if (FileHelper.isDiagonalStep(path[i], path[i - 1]))
            {
                if (diag % 2 == 0)
                {
                    sum += 1;
                }

                diag++;
            }

            sum += graph.GetCost(path[i], path[i - 1]);
        }

        return sum;
    }
}