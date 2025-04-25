#include "stdafx.h"
#include "NavGrid.h"
#include "NavMesh.h"
#include <tge\engine.h>

#undef max
#undef min
//CODE USED IS INSPIRED FROM https://github.com/DRossen/Pathfinding/tree/main

void NavGrid::Init(const float aCellSize, const Vector2f aMin, const Vector2f aMax, std::vector<Node>& aNodes)
{
    myCellSize = aCellSize;
    myMin = aMin;
    myMax = aMax;

    //Compute grid size based on known bounds (1600x900)
    myGridSize = { (int)(std::abs(aMax.x - aMin.x) / myCellSize) + 1,
                   (int)(std::abs(aMax.y - aMin.y) / myCellSize) + 1 };
    myCells.resize(myGridSize.x * myGridSize.y);

    //Assign nodes to grid cells
    for (Node& node : aNodes)
    {
        Vector2f min = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        Vector2f max = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

        for (const auto& vertex : node.myVertices)
        {
            if (vertex.x < min.x) min.x = vertex.x;
            if (vertex.y < min.y) min.y = vertex.y;
            if (vertex.x > max.x) max.x = vertex.x;
            if (vertex.y > max.y) max.y = vertex.y;
        }

        Vector2i minCord = GetCoordinate(min);
        Vector2i maxCord = GetCoordinate(max);

        for (int y = minCord.y; y <= maxCord.y; y++)
        {
            for (int x = minCord.x; x <= maxCord.x; x++)
            {
                int index = x + y * myGridSize.x;
                if (index >= 0 && index < myCells.size()) //Ensure within bounds
                {
                    myCells[index].myNodeData.push_back(&node);
                }
            }
        }
    }
}

int NavGrid::GetCellIndex(const Vector2f aPos) const
{
    Vector2i coordinate = GetCoordinate(aPos);
    return coordinate.x + coordinate.y * myGridSize.x;
}

Vector2i NavGrid::GetCoordinate(Vector2f aPos) const
{
    float relativeX = aPos.x - myMin.x;
    float relativeY = aPos.y - myMin.y;
    return Vector2i((int)(relativeX / myCellSize), (int)(relativeY / myCellSize));
}

std::vector<Node*> NavGrid::GetNodesAtPos(const Vector2f aPos) const
{
    int index = GetCellIndex(aPos);
    if (index < 0 || index >= myCells.size())
        return std::vector<Node*>();

    return myCells[index].myNodeData;
}

void NavGrid::RenderGrid()
{
#ifdef _DEBUG
    Tga::Color gridColor(1.0f, 1.0f, 1.0f, 0.1f);
    const auto& engine = *Tga::Engine::GetInstance();
    Tga::DebugDrawer& debugDrawer = engine.GetDebugDrawer();

    for (int y = 0; y < myGridSize.y; y++)
    {
        for (int x = 0; x < myGridSize.x; x++)
        {
            Vector2f topLeft = { myMin.x + x * myCellSize, myMin.y + y * myCellSize };
            Vector2f topRight = { myMin.x + (x + 1) * myCellSize, myMin.y + y * myCellSize };
            Vector2f bottomLeft = { myMin.x + x * myCellSize, myMin.y + (y + 1) * myCellSize };
            Vector2f bottomRight = { myMin.x + (x + 1) * myCellSize, myMin.y + (y + 1) * myCellSize };

            debugDrawer.DrawLine(topLeft, topRight, gridColor);
            debugDrawer.DrawLine(topLeft, bottomLeft, gridColor);
            debugDrawer.DrawLine(bottomLeft, bottomRight, gridColor);
            debugDrawer.DrawLine(topRight, bottomRight, gridColor);
        }
    }
#endif // _DEBUG
}
