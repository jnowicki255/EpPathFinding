using C5;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace EpPathFinding.PathFinder
{
    public class CostPointFinder 
    {
        public static List<GridPos> FindPath(JumpPointParam param)
        {
            IntervalHeap<Node> openList = param.openList;
            Node startNode = param.StartNode;
            Node endNode = param.EndNode;
            Node node;
            bool revertEndNodeWalkable = false;

            // set the `g` and `f` value of the start node to be 0
            startNode.startToCurNodeLen = 0;
            startNode.heuristicCurNodeToEndLen = 0;

            // push the start node into the open list
            openList.Add(startNode);
            startNode.isOpened = true;

            if (param.CurEndNodeUnWalkableTreatment == EndNodeUnWalkableTreatment.ALLOW && !param.SearchGrid.IsWalkableAt(endNode.x, endNode.y))
            {
                param.SearchGrid.SetWalkableAt(endNode.x, endNode.y, true);
                revertEndNodeWalkable = true;
            }

            // while the open list is not empty
            while (openList.Count > 0)
            {
                // pop the position of node which has the minimum `f` value.
                node = openList.DeleteMin();
                node.isClosed = true;

                if(node.Equals(endNode))
                {
                    if (revertEndNodeWalkable)
                        param.SearchGrid.SetWalkableAt(endNode.x, endNode.y, false);

                    return Node.Backtrace(node); // rebuilding path
                }

                IdentifySuccessors(param, node);
            }

            if (revertEndNodeWalkable)
                param.SearchGrid.SetWalkableAt(endNode.x, endNode.y, false);

            // fail to find the path
            return new List<GridPos>();
        }

        private static void IdentifySuccessors(JumpPointParam param, Node node)
        {
            HeuristicDelegate heuristic = param.HeuristicFunc;
            IntervalHeap<Node> openList = param.openList;
            int endX = param.EndNode.x;
            int endY = param.EndNode.y;
            GridPos neighbor;
            GridPos jumpPoint;
            Node jumpNode;

            List<GridPos> neighbors = FindNeighbors(param, node);
            for (int i = 0; i < neighbors.Count; i++)
            {
                neighbor = neighbors[i];

                if (param.CurIterationType == IterationType.RECURSIVE)
                {
                    //jumpPoint = Jump(param, neighbor.x, neighbor.y, node.x, node.y);
                }
            }
        }

        private static List<GridPos> FindNeighbors(JumpPointParam param, Node node)
        {
            Node parent = node.parent as Node;
            int x = node.x;
            int y = node.y;
            int pX, pY, dX, dY;
            List<GridPos> neighbors = new List<GridPos>();
            List<Node> neighborNodes;
            Node neighborNode;

            if (parent != null)
            {

            }
            else
            {
                neighborNodes = param.SearchGrid.GetNeighbors(node, param.DiagonalMovement);
            }
        }
    }
}
