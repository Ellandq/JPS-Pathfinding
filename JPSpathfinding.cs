using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JPSPathfinder
{
    private PathNode[,] nodeGrid;
    private List<PathNode> openList;
    private List<PathNode> closedList;

    // Initialize the node grid, open list, and closed list
    public JPSPathfinder(PathNode[,] nodeGrid)
    {
        this.nodeGrid = nodeGrid;
        openList = new List<PathNode>();
        closedList = new List<PathNode>();
    }

    // Find the shortest path between the start and end positions using the JPS algorithm
    public List<PathNode> FindPath(Vector2Int startPos, Vector2Int endPos)
    {
        // Get the start and end nodes
        PathNode startNode = nodeGrid[startPos.x, startPos.y];
        PathNode endNode = nodeGrid[endPos.x, endPos.y];

        // Set the gCost and hCost of all nodes to large numbers, and set their cameFromCell values to null
        for (int x = 0; x < nodeGrid.GetLength(0); x++)
        {
            for (int y = 0; y < nodeGrid.GetLength(1); y++)
            {
                PathNode node = nodeGrid[x, y];
                node.gCost = int.MaxValue;
                node.hCost = int.MaxValue;
                node.fCost = int.MaxValue;
                node.cameFromCell = null;
            }
        }

        // Set the gCost of the start node to 0 and calculate its hCost and fCost
        startNode.gCost = 0;
        startNode.hCost = CalculateDistanceCost(startNode, endNode);
        startNode.fCost = startNode.gCost + startNode.hCost;

        // Add the start node to the open list
        openList.Add(startNode);

        // Continue searching for a path until the open list is empty or the end node has been added to the closed list
        while (openList.Count > 0)
        {
            // Sort the open list by fCost
            openList.Sort((node1, node2) => node1.fCost.CompareTo(node2.fCost));

            // Get the node with the lowest fCost from the open list
            PathNode currentNode = openList[0];
            openList.Remove(currentNode);
            closedList.Add(currentNode);

            // If the current node is the end node, retrace the path and return it
            if (currentNode == endNode)
            {
                return RetracePath(startNode, endNode);
            }

            // Check the neighboring nodes of the current node
            foreach (PathNode neighbor in GetNeighboringNodes(currentNode))
            {
                // Skip the neighbor if it is not walkable or if it is already in the closed list
                if (!neighbor.isWalkable || closedList.Contains(neighbor))
                {
                    continue;
                }

                // Calculate the gCost of the neighbor
                int newMovementCostToNeighbor = currentNode.gCost + CalculateDistanceCost(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost)
                {
                    // Update the neighbor's gCost and set its cameFromCell value to the current node
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.cameFromCell = currentNode;

                    // Calculate the neighbor's hCost and fCost
                    neighbor.hCost = CalculateDistanceCost(neighbor, endNode);
                    neighbor.fCost = neighbor.gCost + neighbor.hCost;

                    // Add the neighbor to the open list if it is not already there
                    if (!openList.Contains(neighbor))
                    {
                        openList.Add(neighbor);
                    }
                }
            }
        }

        // Return null if no path was found
        return null;
    }

    // Retrace the path from the start to the end node
    private List<PathNode> RetracePath(PathNode startNode, PathNode endNode)
    {
        List<PathNode> path = new List<PathNode>();
        PathNode currentNode = endNode;

        // Go through the cameFromCell values and add the nodes to the path list
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.cameFromCell;
        }

        // Reverse the path list and return it
        path.Reverse();
        return path;
    }

    // Get the neighboring nodes of the given node
    private List<PathNode> GetNeighboringNodes(PathNode node)
    {
        List<PathNode> neighbors = new List<PathNode>();

        // Add the directly adjacent neighbors
        if (node.gridPosX > 0)
        {
            neighbors.Add(nodeGrid[node.gridPosX - 1, node.gridPosY]);
        }
        if (node.gridPosX < nodeGrid.GetLength(0) - 1){
            // Calculate the gCost of the neighbor
            int newMovementCostToNeighbor = currentNode.gCost + CalculateDistanceCost(currentNode, neighbor);
            if (newMovementCostToNeighbor < neighbor.gCost)
            {
                // Update the neighbor's gCost and set its cameFromCell value to the current node
                neighbor.gCost = newMovementCostToNeighbor;
                neighbor.cameFromCell = currentNode;

                // Calculate the neighbor's hCost and fCost
                neighbor.hCost = CalculateDistanceCost(neighbor, endNode);
                neighbor.fCost = neighbor.gCost + neighbor.hCost;

                // Add the neighbor to the open list if it is not already there
                if (!openList.Contains(neighbor))
                {
                    openList.Add(neighbor);
                }
            }
        }
        // Return null if no path was found
        return null;
    }

    // Retrace the path from the start to the end node
    private List<PathNode> RetracePath(PathNode startNode, PathNode endNode)
    {
        List<PathNode> path = new List<PathNode>();
        PathNode currentNode = endNode;

        // Go through the cameFromCell values and add the nodes to the path list
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.cameFromCell;
        }

        // Reverse the path list and return it
        path.Reverse();
        return path;
    }

    // Get the forced neighboring nodes of the given node
    private List<PathNode> GetNeighboringNodes(PathNode node, PathNode endNode)
    {
        List<PathNode> neighbors = new List<PathNode>();

        // Check the directly adjacent neighbors
        if (node.gridPosX > 0)
        {
            PathNode neighbor = nodeGrid[node.gridPosX - 1, node.gridPosY];
            if (neighbor.isWalkable)
            {
                neighbors.Add(neighbor);
            }
        }
        if (node.gridPosX < nodeGrid.GetLength(0) - 1)
        {
            PathNode neighbor = nodeGrid[node.gridPosX + 1, node.gridPosY];
            if (neighbor.isWalkable)
            {
                neighbors.Add(neighbor);
            }
        }
        if (node.gridPosY > 0)
        {
            PathNode neighbor = nodeGrid[node.gridPosX, node.gridPosY - 1];
            if (neighbor.isWalkable)
            {
                neighbors.Add(neighbor);
            }
        }
        if (node.gridPosY < nodeGrid.GetLength(1) - 1)
        {
            PathNode neighbor = nodeGrid[node.gridPosX, node.gridPosY + 1];
            if (neighbor.isWalkable)
            {
                neighbors.Add(neighbor);
            }
        }

        // Check the diagonal neighbors
        if (node.gridPosX > 0 && node.gridPosY > 0)
        {
            PathNode neighbor = nodeGrid[node.gridPosX - 1, node.gridPosY - 1];
            if (neighbor.isWalkable)
            {
                // Check if the node is a forced neighbor along the x-axis
                if ((node.gridPosX < endNode.gridPosX && !nodeGrid[node.gridPosX + 1, node.gridPosY].isWalkable) ||
                    (node.gridPosX > endNode.gridPosX && !nodeGrid[node.gridPosX - 1, node.gridPosY].isWalkable))
                {
                    neighbors.Add(neighbor);
                }
                // Check if the node is a forced neighbor along the y-axis
                else if ((node.gridPosY < endNode.gridPosY && !nodeGrid[node.gridPosX, node.gridPosY + 1].isWalkable) ||
                        (node.gridPosY > endNode.gridPosY && !nodeGrid[node.gridPosX, node.gridPosY - 1].isWalkable))
                {
                    neighbors.Add(neighbor);
                }
            }
        }
        if (node.gridPosX < nodeGrid.GetLength(0) - 1 && node.gridPosY > 0)
        {
            PathNode neighbor = nodeGrid[node.gridPosX + 1, node.gridPosY - 1];
            if (neighbor.isWalkable)
            {
                // Check if the node is a forced neighbor along the x-axis
                if ((node.gridPosX < endNode.gridPosX && !nodeGrid[node.gridPosX + 1, node.gridPosY].isWalkable) ||
                    (node.gridPosX > endNode.gridPosX && !nodeGrid[node.gridPosX - 1, node.gridPosY].isWalkable))
                {
                    neighbors.Add(neighbor);
                }
                // Check if the node is a forced neighbor along the y-axis
                else if ((node.gridPosY < endNode.gridPosY && !nodeGrid[node.gridPosX, node.gridPosY + 1].isWalkable) ||
                        (node.gridPosY > endNode.gridPosY && !nodeGrid[node.gridPosX, node.gridPosY - 1].isWalkable))
                {
                    neighbors.Add(neighbor);
                }
            }
        }
        return neighbors;
    }

    // Calculate the distance cost between two nodes using the Manhattan distance formula
    private int CalculateDistanceCost(PathNode nodeA, PathNode nodeB)
    {
        int distanceX = Mathf.Abs(nodeA.gridPosX - nodeB.gridPosX);
        int distanceY = Mathf.Abs(nodeA.gridPosY - nodeB.gridPosY);

        int distanceCost;
        if (distanceX > distanceY)
        {
            distanceCost = 14 * distanceY + 10 * (distanceX - distanceY);
        }
        else
        {
            distanceCost = 14 * distanceX + 10 * (distanceY - distanceX);
        }

        // Multiply the distance cost by the movement cost of the cell
        distanceCost *= nodeB.movementCost;

        return distanceCost;
    }
}

