#include "AIAgent.h"
#include <iostream>
#include <imgui\imgui.h>
#include "navmesh.h"
#include "FireAtTarget.h"
#include "Revive.h"
#include "Blackboard.h"

AIAgent::AIAgent(int aID, Team aTeam, std::shared_ptr<Navmesh> aNavmesh, Tga::Vector2f& aPosition)
    : myAgentID(aID), myTeam(aTeam), myNavmesh(aNavmesh), myPosition(aPosition)
{
    myHealth = 100.0f;
    myAmmo = 10;

    const auto& engine = *Tga::Engine::GetInstance();
    Tga::Vector2ui intResolution = engine.GetRenderSize();
    Tga::Vector2f resolution = { static_cast<float>(intResolution.x), static_cast<float>(intResolution.y) };

    const wchar_t* texturePath = (myTeam == Team::Blue) ? L"Sprites/SoldierBlue.png" : L"Sprites/SoldierRed.png";
    myRenderData.sSharedDataAlive.myTexture = engine.GetTextureManager().GetTexture(texturePath);
    myRenderData.sInstanceData.myRotation = (myTeam == Team::Blue) ? 1.57f : -1.57f;
    myRenderData.sSharedDataDead.myTexture = engine.GetTextureManager().GetTexture(L"Sprites/DeadSoldier.png");
    myRenderData.sInstanceData.myPivot = { 0.5f, 0.5f };
    myRenderData.sInstanceData.myPosition = myPosition;
    myRenderData.sInstanceData.mySize = { 30.0f, 40.5f };

    Tga::Color textColor = (myTeam == Team::Blue) ? Tga::Color{ 0, 0, 1, 1 } : Tga::Color{ 1, 0, 0, 1 };

    Tga::Vector2f textPosHealth = { myPosition.x, myPosition.y + 25.0f };
    Tga::Vector2f textPosAmmo = { myPosition.x, myPosition.y + 15.0f };

    myHealthText = Tga::Text{ L"Text/arial.ttf", Tga::FontSize_10 };
    myHealthText.SetColor(textColor);
    myHealthText.SetPosition(textPosHealth);

    myAmmoText = Tga::Text{ L"Text/arial.ttf", Tga::FontSize_10 };
    myAmmoText.SetColor(textColor);
    myAmmoText.SetPosition(textPosAmmo);
}
AIAgent::~AIAgent() {}

void AIAgent::Init()
{    
    myHealth = 100.0f;
    myAmmo = 10;
    myIsAlive = true;
    myIsInCombat = false;
    myIsAttackingLeader = false;
    myIsDefendingLeader = false;
    myActionState = ActionState::IDLE;
    myPath.clear();

    Blackboard::GetInstance()->RegisterAgent(myAgentID, myTeam, myPosition);

    const auto& engine = *Tga::Engine::GetInstance();
    const wchar_t* texturePath = (myTeam == Team::Blue) ? L"Sprites/SoldierBlue.png" : L"Sprites/SoldierRed.png";
    myRenderData.sSharedDataAlive.myTexture = engine.GetTextureManager().GetTexture(texturePath);
    myRenderData.sInstanceData.myRotation = (myTeam == Team::Blue) ? 1.57f : -1.57f;
    myRenderData.sInstanceData.mySize = { 30.0f, 40.5f };
}

void AIAgent::AddAction(std::shared_ptr<GOAPAction> aActon)
{
    myAvailableActions.push_back(aActon);
}

void AIAgent::AddGoal(std::shared_ptr<GOAPGoal> aGoal)
{
    myAvailableGoals.push_back(aGoal);
}

void AIAgent::SetCurrentAction(std::shared_ptr<GOAPAction> aAction)
{
    if (auto fireAction = std::dynamic_pointer_cast<FireAtTarget>(aAction))
    {
        Team enemyTeam = (myTeam == Team::Blue) ? Team::Red : Team::Blue;
        auto target = Blackboard::GetInstance()->GetClosestEnemy(myPosition, enemyTeam);

        if (!target || !target->IsAlive())
        {
            myCurrentAction = nullptr;
            myActionState = ActionState::IDLE;
            return;
        }
        fireAction->SetTarget(target);
    }
    else if (auto reviveAction = std::dynamic_pointer_cast<Revive>(aAction))
    {
        auto target = Blackboard::GetInstance()->GetClosestDeadAlly(myPosition, myTeam);
        if (target)
        {
            reviveAction->SetTarget(target);
            reviveAction->SetActionPosition(target->GetPosition());
        }
    }

    myCurrentAction = aAction;
    myActionState = ActionState::IDLE;
}

bool AIAgent::HasCompletedAction() const
{
    return myActionState == ActionState::IDLE;
}

void AIAgent::UpdateCombatData(bool aInCombat, bool aAtkLeader, bool aDefLeader)
{
    myIsInCombat = aInCombat;
    myIsAttackingLeader = aAtkLeader;
    myIsDefendingLeader = aDefLeader;
}

void AIAgent::Update(const float aDeltaTime)
{
    auto blackboard = Blackboard::GetInstance();
    if (!myIsAlive)
        return;
    if (myHealth <= 0)
    {
        const auto& engine = *Tga::Engine::GetInstance();
        myIsAlive = false;
        myPath.clear();
        blackboard->RemoveAgent(myAgentID, myTeam);
       
        if (myTeam == Team::Blue)
        {
            myRenderData.sSharedDataAlive.myTexture = engine.GetTextureManager().GetTexture(L"Sprites/DeadSoldierBlue.png");
        }
        else
        {
            myRenderData.sSharedDataAlive.myTexture = engine.GetTextureManager().GetTexture(L"Sprites/DeadSoldierRed.png");
        }

        myRenderData.sInstanceData.mySize = Tga::Vector2f{ 27.0f, 35.5f };
        myActionState = ActionState::IDLE;
        return;
    }
    
    
    if (myCurrentAction != nullptr)
    {
        float requiredDistance = myCurrentAction->GetExecutionRange();
        float currentDistance = (myPosition - myCurrentAction->GetActionPosition()).Length();

        if (currentDistance > requiredDistance)
            myActionState = ActionState::MOVING;
        else
            myActionState = ActionState::EXECUTING;
    }
    ApplySeparation();
    switch (myActionState)
    {
    case ActionState::IDLE:
    {
        break;
    }
    case ActionState::EXECUTING:
    {
        UpdateRotation(myCurrentAction->GetActionPosition());
        ExecuteAction(aDeltaTime, myCurrentAction);
        break;
    }
        
    case ActionState::MOVING:
    {               
        MoveToAction(aDeltaTime);
        break;
    }
        
    default:
        break;
    }
    
    blackboard->UpdateAgentPosition(myAgentID, myTeam, myPosition);
    Tga::Vector2f textPosHealth = myPosition;
    Tga::Vector2f textPosAmmo = myPosition;
    textPosHealth.y += 25.0f;
    textPosAmmo.y += 15.0f;

    myHealthText.SetText("HP: " + std::to_string((int)myHealth));
    myAmmoText.SetText("Am: " + std::to_string(myAmmo));
    myHealthText.SetPosition(textPosHealth);
    myAmmoText.SetPosition(textPosAmmo);
    myHealthText.Render();
    myAmmoText.Render();
}

void AIAgent::Render()
{
    const auto& engine = *Tga::Engine::GetInstance();
    auto sDrawer = &engine.GetGraphicsEngine().GetSpriteDrawer();
    sDrawer->Draw(myRenderData.sSharedDataAlive, myRenderData.sInstanceData);
}

void AIAgent::ExecuteAction(const float aDeltaTime, std::shared_ptr<GOAPAction> aAction)
{
    if (aAction != nullptr)
    {
        if (aAction->Go(aDeltaTime, shared_from_this()) == true)
        {
            std::cout << "Agent " << myAgentID << " finished action: " << myCurrentAction->GetName() << "\n";
            myCurrentAction = nullptr;
            myActionState = ActionState::IDLE;
        }
    }   
}

void AIAgent::MoveToAction(const float aDeltaTime)
{ 
    myPathRecheckTimer += aDeltaTime;

    const Tga::Vector2f currentTargetPos = myCurrentAction->GetActionPosition();
    bool targetMovedSignificantly = (currentTargetPos - myLastPathTarget).Length() >= 20.0f;
    bool shouldRepath = false;

    if (myPath.empty())
    {
        shouldRepath = true;
    }
    else if (myPathRecheckTimer >= myPathRecheckInterval && targetMovedSignificantly)
    {
        shouldRepath = true;
        myPathRecheckTimer = 0.0f;
    }

    if (shouldRepath)
    {
        bool isInNavmesh = myNavmesh->GetNodeIndexFromPoint(currentTargetPos) != -1;

        Tga::Vector2f validTarget = isInNavmesh ? currentTargetPos : myLastValidNavPoint;
        if (isInNavmesh)
            myLastValidNavPoint = currentTargetPos;

        std::vector<Tga::Vector2f> newPath = myNavmesh->PickNode(myPosition, validTarget);

        if (!newPath.empty())
        {
            myPath = newPath;
            myCurrentPathIndex = 1;
            myLastPathTarget = currentTargetPos;
        }
        else
        {
            std::cout << "Invalid path for AIAgent " << myAgentID << ": path too short or not found.\n";
        }
    }

    const auto& engine = *Tga::Engine::GetInstance();
    Tga::DebugDrawer& debugDrawer = engine.GetDebugDrawer();

    if (!myPath.empty() && myCurrentPathIndex < myPath.size())
    {
        Tga::Vector2f currentPosition = myPosition;
        Tga::Vector2f targetPosition = myPath[myCurrentPathIndex];

        for (int p = 0; p < static_cast<int>(myPath.size()) - 1; ++p) // rendering path
        {
            Tga::Vector2f start = myPath[p];
            Tga::Vector2f end = myPath[p + 1];
            if (myTeam == Team::Blue)
                debugDrawer.DrawLine(start, end, { 0, 0, 1, 1 });
            else
                debugDrawer.DrawLine(start, end, { 1, 0, 0, 1 });
        }

        Tga::Vector2f direction = (targetPosition - currentPosition);
        float distanceToTarget = direction.Length();        

        if (distanceToTarget > 0.0001f)
        {
            direction.Normalize();
            float moveDistance = myMoveSpeed * aDeltaTime;

            if (moveDistance >= distanceToTarget)
            {
                myPosition = targetPosition;              
                myRenderData.sInstanceData.myPosition = myPosition;

                if (myCurrentPathIndex < myPath.size() - 1)
                {
                    myCurrentPathIndex++;
                }
                else
                {
                    myPath.clear();
                    std::cout << "Agent " << myAgentID << " finished the path to the action!\n";
                    myActionState = ActionState::EXECUTING;
                }
            }
            else
            {
                myPosition += direction * moveDistance;
                myRenderData.sInstanceData.myPosition = myPosition;
            }
        } 
        UpdateRotation(targetPosition);
    }
}

void AIAgent::ApplySeparation()
{
    const float separationRadius = 15.0f;
    Tga::Vector2f separationForce = { 0.0f, 0.0f };
    int neighborCount = 0;

    Blackboard* blackboard = Blackboard::GetInstance();

    std::shared_ptr<AISquad> friendlySquad = blackboard->GetSquadByTeam(myTeam);
    std::shared_ptr<AISquad> enemySquad = blackboard->GetSquadByTeam(myTeam == Team::Blue ? Team::Red : Team::Blue);

    std::vector<std::shared_ptr<AIAgent>> nearbyAgents;

    if (friendlySquad)
        nearbyAgents.insert(nearbyAgents.end(), friendlySquad->GetSoldiers().begin(), friendlySquad->GetSoldiers().end());

    if (enemySquad)
        nearbyAgents.insert(nearbyAgents.end(), enemySquad->GetSoldiers().begin(), enemySquad->GetSoldiers().end());

    for (const auto& other : nearbyAgents)
    {
        if (other.get() == this || !other->IsAlive())
            continue;

        float dist = (other->GetPosition() - myPosition).Length();
        if (dist > 0.0f && dist < separationRadius)
        {
            Tga::Vector2f away = (myPosition - other->GetPosition()).GetNormalized();
            float strength = (separationRadius - dist) / separationRadius;
            separationForce += away * strength;
            neighborCount++;
        }
    }

    if (neighborCount > 0)
    {
        separationForce /= static_cast<float>(neighborCount);
        Tga::Vector2f candidate = myPosition + separationForce;

        if (myNavmesh->GetNodeIndexFromPoint(candidate) != -1)
        {
            myPosition = candidate;
            myRenderData.sInstanceData.myPosition = myPosition;
        }
    }
}

void AIAgent::UpdateRotation(const Tga::Vector2f& aTarget)
{
    Tga::Vector2f direction = aTarget - myPosition;

    if (direction.LengthSqr() > 0.0001f)
    {
        direction.Normalize();
        float angleRadians = atan2f(direction.y, direction.x);
        angleRadians += 3.14f / 2.0f;
        myRenderData.sInstanceData.myRotation = angleRadians;
    }
}
