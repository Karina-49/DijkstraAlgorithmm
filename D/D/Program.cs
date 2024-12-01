using System;
using System.Collections.Generic;
using System.Linq;

namespace DijkstraAlgorithmDemo
{
    // Класс для представления графа
    public class Graph
    {
        public List<string> Vertices { get; } = new List<string>();
        public Dictionary<string, List<(string, int)>> Edges { get; } = new Dictionary<string, List<(string, int)>>();

        // Добавление вершины
        public void AddVertex(string vertex)
        {
            if (!Vertices.Contains(vertex))
            {
                Vertices.Add(vertex);
                Edges[vertex] = new List<(string, int)>();
            }
        }

        // Добавление ребра
        public void AddEdge(string from, string to, int weight)
        {
            if (!Vertices.Contains(from) || !Vertices.Contains(to))
                throw new ArgumentException("Вершины должны быть добавлены до создания ребра.");

            Edges[from].Add((to, weight));
        }

        // Получение соседей вершины
        public List<(string, int)> GetNeighbors(string vertex)
        {
            return Edges.ContainsKey(vertex) ? Edges[vertex] : new List<(string, int)>();
        }
    }

    // Класс для реализации алгоритма Дейкстры
    public class DijkstraAlgorithm
    {
        public Dictionary<string, int> Distances { get; private set; }
        public Dictionary<string, string> PreviousNodes { get; private set; }

        public void FindShortestPath(Graph graph, string startVertex)
        {
            // Инициализация
            Distances = graph.Vertices.ToDictionary(v => v, v => int.MaxValue);
            PreviousNodes = graph.Vertices.ToDictionary(v => v, v => (string)null);
            Distances[startVertex] = 0;

            var priorityQueue = new SortedSet<(int Distance, string Vertex)>();
            priorityQueue.Add((0, startVertex));

            while (priorityQueue.Count > 0)
            {
                var (currentDistance, currentVertex) = priorityQueue.First();
                priorityQueue.Remove(priorityQueue.First());

                foreach (var (neighbor, weight) in graph.GetNeighbors(currentVertex))
                {
                    var newDistance = currentDistance + weight;

                    if (newDistance < Distances[neighbor])
                    {
                        priorityQueue.Remove((Distances[neighbor], neighbor));
                        Distances[neighbor] = newDistance;
                        PreviousNodes[neighbor] = currentVertex;
                        priorityQueue.Add((newDistance, neighbor));
                    }
                }
            }
        }

        public List<string> GetShortestPath(string endVertex)
        {
            var path = new List<string>();
            for (var current = endVertex; current != null; current = PreviousNodes[current])
                path.Add(current);

            path.Reverse();
            return path;
        }
    }

    // Главный класс программы
    public class Program
    {
        public static void Main(string[] args)
        {
            // Создание графа
            var graph = new Graph();
            graph.AddVertex("A");
            graph.AddVertex("B");
            graph.AddVertex("C");
            graph.AddVertex("D");
            graph.AddVertex("E");
            graph.AddVertex("F");

            graph.AddEdge("A", "B", 3);
            graph.AddEdge("A", "C", 1);
            graph.AddEdge("B", "D", 2);
            graph.AddEdge("C", "D", 4);
            graph.AddEdge("D", "E", 1);
            graph.AddEdge("E", "F", 3);

            // Выполнение алгоритма Дейкстры
            var dijkstra = new DijkstraAlgorithm();
            dijkstra.FindShortestPath(graph, "A");

            // Вывод кратчайших расстояний
            Console.WriteLine("Кратчайшие расстояния от вершины A:");
            foreach (var vertex in graph.Vertices)
            {
                Console.WriteLine($"До {vertex}: {dijkstra.Distances[vertex]}");
            }

            // Вывод кратчайшего пути до вершины F
            var path = dijkstra.GetShortestPath("F");
            Console.WriteLine("\nКратчайший путь до вершины F:");
            Console.WriteLine(string.Join(" -> ", path));
        }
    }
}
