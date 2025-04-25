#include "stdafx.h"
#include "navmesh.h"
#include <fstream>
#include <queue>
#include <cmath>
#include <random>
#include "tge/input/InputManager.h"
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static Tga::Vector2f startPos;
static Tga::Vector2f endPos;
static int navMeshIterationIndex = 0;

bool PointInTriangle(const Tga::Vector2f& aMousePos, const Tga::Vector2f& aVertex1, const Tga::Vector2f& aVertex2, const Tga::Vector2f& aVertex3)
{
	auto triangleArea = [](float x1, float y1, float x2, float y2, float x3, float y3)
	{
		return std::fabs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));
	};

	float areaOrig = triangleArea(aVertex1.myX, aVertex1.myY, aVertex2.myX, aVertex2.myY, aVertex3.myX, aVertex3.myY);
	float area1 = triangleArea(aMousePos.myX, aMousePos.myY, aVertex2.myX, aVertex2.myY, aVertex3.myX, aVertex3.myY);
	float area2 = triangleArea(aVertex1.myX, aVertex1.myY, aMousePos.myX, aMousePos.myY, aVertex3.myX, aVertex3.myY);
	float area3 = triangleArea(aVertex1.myX, aVertex1.myY, aVertex2.myX, aVertex2.myY, aMousePos.myX, aMousePos.myY);

	return std::fabs(areaOrig - (area1 + area2 + area3)) < 1e-2;
}

Navmesh::Navmesh() : myStartNodeIndex(-1), myEndNodeIndex(-1){}

Navmesh::~Navmesh(){}

void Navmesh::Init(const char* aObjFile)
{
	myMesh = LoadMesh(aObjFile);
	CreateNodes();
	CalculateConnections();

	Tga::Vector2f minBounds(0.0f, 0.0f);
	Tga::Vector2f maxBounds(1600.0f, 900.0f);
	const float cellSize = 100.0f;
	myGrid.Init(cellSize, minBounds, maxBounds, myNodes);
}

void Navmesh::RenderNavmesh(Tga::DebugDrawer& debugDrawer, Tga::InputManager* aInput)
{
	debugDrawer;
	if (aInput->IsKeyPressed(VK_F1))
	{
		if (myShouldRenderGrid)
		{
			myShouldRenderGrid = false;
		}
		else
		{
			myShouldRenderGrid = true;
		}
	}
	if (aInput->IsKeyPressed(VK_F2))
	{
		if (myShouldRenderNavmesh)
		{
			myShouldRenderNavmesh = false;
		}
		else
		{
			myShouldRenderNavmesh = true;
		}
	}

	if (myShouldRenderGrid)
	{
		myGrid.RenderGrid();
	}
	
	if (!myShouldRenderNavmesh)
		return;

	for (int nodeIndex = 0; nodeIndex < myMesh.myIndices.size(); nodeIndex += 3)
	{
		int index1 = myMesh.myIndices[nodeIndex + 0];
		int index2 = myMesh.myIndices[nodeIndex + 1];
		int index3 = myMesh.myIndices[nodeIndex + 2];

		Tga::Vector2f vertex1 = myMesh.myVertices[index1];
		Tga::Vector2f vertex2 = myMesh.myVertices[index2];
		Tga::Vector2f vertex3 = myMesh.myVertices[index3];

#ifdef _DEBUG
		Tga::Color navMeshColor(0.f, 1.f, 0.f, 0.1f);
		debugDrawer.DrawLine(vertex1, vertex2, navMeshColor);
		debugDrawer.DrawLine(vertex2, vertex3, navMeshColor);
		debugDrawer.DrawLine(vertex3, vertex1, navMeshColor);
#endif //!_DEBUG

	}		
}

const std::vector<Node>& Navmesh::GetNodes() const
{
	return myNodes;
}

Tga::Vector2f Navmesh::GetRandomNodePos()
{
	std::mt19937 rng;
	rng.seed(std::random_device()());
	std::uniform_int_distribution<int>distribution(16, (int)myNodes.size() - 1);
	int randomNum = distribution(rng);
	return Tga::Vector2f(myNodes[randomNum].myCenter);
}

int Navmesh::GetNodeIndexFromPoint(const Tga::Vector2f aPoint) const
{
	std::vector<Node*> nodes = myGrid.GetNodesAtPos(aPoint);
	int nodeIndex = -1;
	for (auto node : nodes)
	{
		Tga::Vector2f v1 = node->myVertices[0];
		Tga::Vector2f v2 = node->myVertices[1];
		Tga::Vector2f v3 = node->myVertices[2];

		if (PointInTriangle(aPoint, v1, v2, v3)) 
		{
			nodeIndex = node->myNavIndex;
		}
	}
	return nodeIndex;	
}

Tga::Vector2f Navmesh::PickRandomValidPoint(const Tga::Vector2f& aOrigin, float aRadius) const
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> angleDist(0.0f, 6.28318f);
	std::uniform_real_distribution<float> radiusDist(0.0f, aRadius);

	for (int i = 0; i < 20; ++i) //Try 20 times to find a valid point
	{
		float angle = angleDist(gen);
		float dist = radiusDist(gen);

		Tga::Vector2f offset(cos(angle) * dist, sin(angle) * dist);
		Tga::Vector2f candidatePosition = aOrigin + offset;

		if (GetNodeIndexFromPoint(candidatePosition) != -1)
		{
			return candidatePosition;
		}
	}
	return aOrigin;
}


bool FindConnectingVertices(Node* aCurrentNode, const Tga::Vector2f& aStartDir, Tga::Vector2f& aLeftVertex, Tga::Vector2f& aRightVertex)
{
	for (size_t i = 0; i < aCurrentNode->myVertices.size(); ++i) 
	{
		for (size_t j = 0; j < aCurrentNode->myVertices.size(); ++j) 
		{
			if (i == j) continue; //Skip the same vertex
			Tga::Vector2f segment = aCurrentNode->myVertices[j] - aCurrentNode->myVertices[i];

			float dotProduct = aStartDir.x * segment.x + aStartDir.y * segment.y;
			float lengthSquared = segment.x * segment.x + segment.y * segment.y;

			if (lengthSquared > 0) 
			{
				Tga::Vector2f normalizedSegment = segment / std::sqrt(lengthSquared);
				if (fabs(dotProduct / std::sqrt(lengthSquared)) > 0.9f) 
				{
					aLeftVertex = aCurrentNode->myVertices[i];
					aRightVertex = aCurrentNode->myVertices[j];
					return true;
				}
			}
		}
	}
	return false;
}

std::vector<Tga::Vector2f> Navmesh::FindPath(const Tga::Vector2f& aStart, const Tga::Vector2f& aEnd, const int aStartIndex, const int aEndIndex)
{
	std::priority_queue<Score> openSet;

	if (aStartIndex == -1 || aEndIndex == -1)
	{
		myShouldUseNavmesh = true;
		return {};  //Return if no valid start or end node was found
	}

	if (aStartIndex == aEndIndex) //If AI is in the same navmesh triangle as the target
	{
		return { aEnd };
	}

	Node* startNode = &myNodes[aStartIndex];
	Node* endNode = &myNodes[aEndIndex];

	for (auto& node : myNodes) {
		node.gCost = std::numeric_limits<float>::infinity();
		node.parent = nullptr;
		node.inOpenSet = false;
	}

	startNode->gCost = 0;
	startNode->inOpenSet = false;
	Score score;
	score.myNode = startNode;
	score.score = 0;

	openSet.push(score);

	while (!openSet.empty())
	{
		Score currentScore = openSet.top();
		openSet.pop();
		Node* current = currentScore.myNode;
		current->inOpenSet = true;

		if (current == endNode)
		{
			//Path found
			std::vector<Node*> path;
			while (current != nullptr)
			{
				path.push_back(current);
				current = current->parent;
			}
			std::reverse(path.begin(), path.end());

			return FindFunnel(path, aStart, aEnd); //Returns a smoother path
		}

		for (auto neighbor : GetNeighbors(current))
		{
			float gCostHolder = current->gCost + (current->myCenter - neighbor->myCenter).Length();

			if (gCostHolder <= neighbor->gCost)
			{
				neighbor->gCost = gCostHolder;
				neighbor->weight = gCostHolder + (neighbor->myCenter - aEnd).Length();
				neighbor->parent = current;

				if (!neighbor->inOpenSet)
				{
					Score value;
					value.myNode = neighbor;
					value.score = neighbor->weight;
					openSet.push(value);
				}
			}
		}
	}
	std::cout << "No path found.\n";
	return {};
} 

bool Navmesh::IsLeftOfLine(const Tga::Vector2f& aApex, const Tga::Vector2f& aEdgeStart, const Tga::Vector2f& aPoint)
{
	const float epsilon = 0.00001f;
	Tga::Vector2f edge = aEdgeStart - aApex;
	Tga::Vector2f toPoint = aPoint - aApex;
	float cross = edge.x * toPoint.y - edge.y * toPoint.x;
	return cross > epsilon;
}

bool Navmesh::IsRightOfLine(const Tga::Vector2f& aApex, const Tga::Vector2f& aEdgeStart, const Tga::Vector2f& aPoint)
{
	const float epsilon = 0.00001f;
	Tga::Vector2f edge = aEdgeStart - aApex;
	Tga::Vector2f toPoint = aPoint - aApex;
	float cross = edge.x * toPoint.y - edge.y * toPoint.x;
	return cross < -epsilon;
}

std::vector<Tga::Vector2f> Navmesh::FindFunnel(std::vector<Node*>& aPath, const Tga::Vector2f& aStart, const Tga::Vector2f& aEnd)
{
	std::vector<Tga::Vector2f> funnelPath;
	if (aPath.empty()) return funnelPath;

	myLeftPortals.resize(aPath.size());
	myRightPortals.resize(aPath.size());

	funnelPath.push_back(aStart);
	Tga::Vector2f apex = funnelPath.back();
	float epsilon = 0.001f;

	for (int i = 0; i < aPath.size() - 1; i++) {
		Tga::Vector2f sharedEdge[2];
		int sharedCount = 0;

		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				Tga::Vector2f& v1 = aPath[i]->myVertices[j];
				Tga::Vector2f& v2 = aPath[i + 1]->myVertices[k];

				if (fabs(v1.x - v2.x) < epsilon &&
					fabs(v1.y - v2.y) < epsilon)
				{
					sharedEdge[sharedCount++] = v1;
					if (sharedCount == 2) break;
				}
			}
			if (sharedCount == 2) break;
		}

		if (sharedCount == 2) {
			Tga::Vector2f nodeDirection = (aPath[i + 1]->myCenter - aPath[i]->myCenter).GetNormalized();
			if (IsLeftOfLine(aPath[i]->myCenter, nodeDirection, sharedEdge[0])) {
				myLeftPortals[i + 1] = sharedEdge[0];
				myRightPortals[i + 1] = sharedEdge[1];
			}
			else {
				myLeftPortals[i + 1] = sharedEdge[1];
				myRightPortals[i + 1] = sharedEdge[0];
			}

			if (IsRightOfLine(aPath[i]->myCenter, myLeftPortals[i + 1], myRightPortals[i + 1])) {
				std::swap(myLeftPortals[i + 1], myRightPortals[i + 1]);
			}
		}
	}

	myRightPortals.emplace_back(aEnd);
	myLeftPortals.emplace_back(aEnd);

	size_t leftIndex = 1, rightIndex = 1;
	for (size_t i = 1; i < myLeftPortals.size(); ++i) {
		const Tga::Vector2f& leftPortal = myLeftPortals[i];
		const Tga::Vector2f& rightPortal = myRightPortals[i];

		if (myLeftPortals[leftIndex].LengthSqr() == leftPortal.LengthSqr() &&
			myRightPortals[rightIndex].LengthSqr() == rightPortal.LengthSqr())
		{
			continue;
		}

		if (IsRightOfLine(apex, leftPortal, myLeftPortals[leftIndex])) {
			if (IsRightOfLine(apex, leftPortal, myRightPortals[rightIndex])) {
				funnelPath.push_back(myRightPortals[rightIndex]);
				apex = myRightPortals[rightIndex];
				leftIndex = rightIndex + 1;
				rightIndex = rightIndex + 1;
				i = rightIndex + 1;

				while (myRightPortals[rightIndex].LengthSqr() == apex.LengthSqr())
				{
					rightIndex++;
				}
				continue;
			}
			leftIndex = i;
		}

		if (IsLeftOfLine(apex, rightPortal, myRightPortals[rightIndex])) {
			if (IsLeftOfLine(apex, rightPortal, myLeftPortals[leftIndex])) {
				funnelPath.push_back(myLeftPortals[leftIndex]);
				apex = myLeftPortals[leftIndex];
				leftIndex = leftIndex + 1;
				rightIndex = leftIndex + 1;
				i = leftIndex + 1;

				while (myLeftPortals[leftIndex].LengthSqr() == apex.LengthSqr())
				{
					leftIndex++;
				}
				continue;
			}
			rightIndex = i;
		}
	}
	funnelPath.push_back(aEnd);
	return funnelPath;
}

float Navmesh::CalculateDistance(const Tga::Vector2f& aStart, const Tga::Vector2f& aEnd)
{
	float distX = aEnd.myX - aStart.myX;
	float distY = aEnd.myY - aStart.myY;
	return std::sqrt(distX * distX + distY * distY);
}

std::vector<Node*> Navmesh::GetNeighbors(Node* aNode)
{
	std::vector<Node*> neighbors;
	for (int connection : aNode->myConnections)
	{
		if (connection != -1)
		{
			neighbors.push_back(&myNodes[connection]);
		}
	}
	return neighbors;
}

std::vector<Tga::Vector2f> Navmesh::PickNode(const Tga::Vector2f& aStartPos, const Tga::Vector2f& aMousePos)
{
	int startNodeIndex = GetNodeIndexFromPoint(aStartPos);
	int endNodeIndex = GetNodeIndexFromPoint(aMousePos);

	if (startNodeIndex == -1 || endNodeIndex == -1)
	{
		std::cout << "Start or end node not found. StartNode: " << startNodeIndex << ", EndNode: " << endNodeIndex << "\n";
		return {};
	}

	std::vector<Tga::Vector2f> path = FindPath(aStartPos, aMousePos, startNodeIndex, endNodeIndex);

	if (path.empty() || !(path.back() == aMousePos))
	{
		path.push_back(aMousePos);
	}
	return path;	
}

void Navmesh::CreateNodes()
{
	for (int nodeIndex = 0; nodeIndex < myMesh.myIndices.size(); nodeIndex += 3)
	{
		Node newNode;
		newNode.myIndices[0] = myMesh.myIndices[nodeIndex + 0];
		newNode.myIndices[1] = myMesh.myIndices[nodeIndex + 1];
		newNode.myIndices[2] = myMesh.myIndices[nodeIndex + 2];

		newNode.myCenter = (myMesh.myVertices[newNode.myIndices[0]] +
			myMesh.myVertices[newNode.myIndices[1]] +
			myMesh.myVertices[newNode.myIndices[2]]) / 3.f;

		newNode.myVertices[0] = myMesh.myVertices[newNode.myIndices[0]];
		newNode.myVertices[1] = myMesh.myVertices[newNode.myIndices[1]];
		newNode.myVertices[2] = myMesh.myVertices[newNode.myIndices[2]];

		newNode.myNavIndex = static_cast<int>(myNodes.size());
		myNodes.push_back(newNode);
	}
}

void Navmesh::CalculateConnections()
{
	const float epsilon = 0.0001f;

	for (int i = 0; i < myNodes.size(); ++i)
	{
		Node& node1 = myNodes[i];

		for (int j = i + 1; j < myNodes.size(); ++j)
		{
			Node& node2 = myNodes[j];

			int sharedVertices = 0;

			for (const auto& v1 : node1.myVertices)
			{
				for (const auto& v2 : node2.myVertices)
				{
					if (fabs(v1.x - v2.x) < epsilon && fabs(v1.y - v2.y) < epsilon)
					{
						sharedVertices++;
						break;
					}
				}
			}

			if (sharedVertices == 2)
			{
				for (int k = 0; k < 3; ++k)
				{
					if (node1.myConnections[k] == -1)
					{
						node1.myConnections[k] = j;
						node1.LocalNodes[k] = &myNodes[j];
						break;
					}
				}

				for (int k = 0; k < 3; ++k)
				{
					if (node2.myConnections[k] == -1)
					{
						node2.myConnections[k] = i;
						node2.LocalNodes[k] = &myNodes[i];
						break;
					}
				}
			}
		}
	}
}

Mesh LoadMesh(const char* aObjFile)
{
	Mesh mesh;
	std::fstream meshLoader;
	meshLoader.open(aObjFile, std::ios_base::in);
	assert(meshLoader.is_open() && "Error! Couldn't open file!");

	const float scaleFactor = 1.0f;
	const float offsetX = 800.0f;  //Shift from (-800, 800) → (0, 1600)
	const float offsetY = 450.0f;  //Shift from (-450, 450) → (0, 900)

	std::string line;
	while (std::getline(meshLoader, line))
	{
		std::replace(line.begin(), line.end(), ',', '.');
		std::stringstream ss(line);
		std::string reader;
		ss >> reader;

		if (reader == "v")
		{
			float x, y, z;
			ss >> x >> y >> z;

			mesh.myVertices.push_back({ (x * scaleFactor) + offsetX, (z * scaleFactor) + offsetY });
		}
		else if (reader == "f")
		{
			int index1, index2, index3;
			std::string vertexData;

			ss >> vertexData;
			std::stringstream vertexStream1(vertexData);
			std::getline(vertexStream1, vertexData, '/');
			index1 = std::stoi(vertexData) - 1;

			ss >> vertexData;
			std::stringstream vertexStream2(vertexData);
			std::getline(vertexStream2, vertexData, '/');
			index2 = std::stoi(vertexData) - 1;

			ss >> vertexData;
			std::stringstream vertexStream3(vertexData);
			std::getline(vertexStream3, vertexData, '/');
			index3 = std::stoi(vertexData) - 1;

			mesh.myIndices.push_back(index1);
			mesh.myIndices.push_back(index2);
			mesh.myIndices.push_back(index3);
		}
	}
	return mesh;
}

