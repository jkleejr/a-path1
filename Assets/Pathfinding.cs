using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;   // timer performance gain
using System;

public class Pathfinding : MonoBehaviour
{
    // once found a path, needs to call finished processingpath on pathrequesetmanager script
    PathRequestManager requestManager;

    Grid grid;   // reference to Grid (for world positions as nodes)

    void Awake() {
        requestManager = GetComponent<PathRequestManager>();
        grid = GetComponent<Grid>();
    }

    // starts findpath coroutine
    public void StartFindPath(Vector3 startPos, Vector3 targetPos) {
        StartCoroutine(FindPath(startPos,targetPos));
    }


    // A* algorithm implementation
    IEnumerator FindPath(Vector3 startPos, Vector3 targetPos) {

        Stopwatch sw = new Stopwatch();
        sw.Start();

        Vector3[] waypoints = new Vector3[0];
        bool pathSuccess = false;

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        if (startNode.walkable && targetNode.walkable) {   // pathfinding if the start and target node is walkable

            // create a new Heap of nodes for openset and closedset with grid maxsize
            Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
            HashSet<Node> closedSet = new HashSet<Node>();

            // add starting node
            openSet.Add(startNode);

            // start loop
            while (openSet.Count > 0) {
                // get current node with lowest fcost and remove from openset
                Node currentNode = openSet.RemoveFirst();
                
                closedSet.Add(currentNode);

                if (currentNode == targetNode) {  // found the path 
                    sw.Stop();
                    print("path found: " + sw.ElapsedMilliseconds + " ms");

                    pathSuccess = true;
                
                    break; 
                }
                // loop through each neighboring node
                foreach (Node neighbor in grid.GetNeighbors(currentNode)) {
                    if (!neighbor.walkable || closedSet.Contains(neighbor)) { 
                        continue;
                    }
                    // check if new path to neighbor is shorter or neighbor is not in open list
                    int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                    if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor)) {
                        // calculate g cost, h cost to find f cost
                        neighbor.gCost = newMovementCostToNeighbor;
                        neighbor.hCost = GetDistance(neighbor, targetNode);
                        // set parent of neighbor to current node
                        neighbor.parent = currentNode;
                        // check if neighbor is in open set, and add it if not
                        if(!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                    }
                }
            }
        }
        yield return null;      // wait 1 frame before returning
        if (pathSuccess) {
            waypoints = RetracePath(startNode, targetNode);
        }
        requestManager.FinishedProcessingPath(waypoints, pathSuccess);
    }

    // if current node = target node, need to retrace steps to get the path from start node to end node
    Vector3[] RetracePath (Node startNode, Node endNode) { 
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode) {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        Vector3[] waypoints = SimplifyPath(path);
        Array.Reverse(waypoints);     // path.Reverse to get path in the right way
        return waypoints;

    }

    Vector3[] SimplifyPath(List<Node> path) {
        List<Vector3> waypoints = new List<Vector3>();   // create list of vector3 called waypoints
        Vector2 directionOld = Vector2.zero;    // create vector2 to store direction of last 2 nodes

        for (int i = 1; i < path.Count; i ++) { 
            // get direction on x axis between last 2 nodes
            Vector2 directionNew = new Vector2(path[i-1].gridX - path[i].gridX, path[i-1].gridY - path[i].gridY);
            if (directionNew != directionOld) {
                waypoints.Add(path[i].worldPosition);   // add waypoint to waypoints list
            }
            directionOld = directionNew;
        }
        return waypoints.ToArray();
    }

    // first count on x axis, and y axis how many nodes away from target node
    // take lowest number (gives us how many diagonal moves it will take to be either horizontal or vertical alligned with end node)
    // subtract higher number - lower number = how many horizontal moves needed
    int GetDistance(Node nodeA, Node nodeB) {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX-dstY);
        return 14 * dstX + 10 * (dstY-dstX);
        }
    
}

