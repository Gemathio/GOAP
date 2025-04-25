#include "AISquad.h"
#include "AIAgent.h"
#include "Blackboard.h"
#include "GOAPPlanner.h"
#include "Heal.h"
#include "FireAtTarget.h"
#include "PickAmmo.h"
#include "EventManager.h"
#include "Retreat.h"
#include "Revive.h"
#include "navmesh.h"
#include <random>

#undef min
#undef SendMessage

AISquad::AISquad(int aID) : mySquadID(aID)
{
    myPlanner = new GOAPPlanner();
}
AISquad::~AISquad()
{
    delete myPlanner;
}
void AISquad::AddSoldier(std::shared_ptr<AIAgent> aSoldier)
{
    mySoldiers.push_back(aSoldier);
}

void AISquad::Init(Team aTeam, std::shared_ptr<Navmesh> aNavmesh)
{
    myTeam = aTeam;
    Blackboard* blackboard = Blackboard::GetInstance();

    //=== GOALS ===
    auto attackEnemyLeader = std::make_shared<GOAPGoal>("Attack Enemy Leader", 1.0f);
    attackEnemyLeader->AddDesire(Conditions::AttackingLeader, { true, 0.2f, 0.0f });
    attackEnemyLeader->AddDesire(Conditions::HasAmmo, { true, 0.3f, 0.9f });
    myAvailableGoals.push_back(attackEnemyLeader);

    auto attackEnemySquad = std::make_shared<GOAPGoal>("Attack Enemy Squad", 0.8f);
    attackEnemySquad->AddDesire(Conditions::InCombat, { true, 0.2f, 0.0f });
    attackEnemySquad->AddDesire(Conditions::HasAmmo, { true, 0.3f, 0.5f });
    myAvailableGoals.push_back(attackEnemySquad);

    auto defendFriendlyLeader = std::make_shared<GOAPGoal>("Defend Friendly Leader", 0.9f);
    defendFriendlyLeader->AddDesire(Conditions::DefendingLeader, { true, 0.1f, 0.0f });
    defendFriendlyLeader->AddDesire(Conditions::InCombat, { true, -0.3f, 0.0f });
    defendFriendlyLeader->AddDesire(Conditions::HasAmmo, { true, 0.2f, 0.5f });
    myAvailableGoals.push_back(defendFriendlyLeader);

    auto keepSquadAlive = std::make_shared<GOAPGoal>("Keep Squad Alive", 0.6f);
    keepSquadAlive->AddDesire(Conditions::LowSES, { false, 0.0f, 0.2f });
    keepSquadAlive->AddDesire(Conditions::LowHealth, { false, 0.3f, 0.3f });
    myAvailableGoals.push_back(keepSquadAlive);

    //=== ACTIONS ===
    {
        auto action = std::make_shared<FireAtTarget>("Fire at Enemy", 1.0f);
        action->AddPrecondition(Conditions::HasAmmo, { true, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::LowHealth, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::InCombat, true);
        action->SetDynamicTarget(DynamicTargetType::ClosestEnemy, myTeam == Team::Blue ? Team::Blue : Team::Red);
        action->SetExecutionRange(200.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<PickAmmo>("Pick Ammo", 1.0f);
        action->AddPrecondition(Conditions::HasAmmo, { false, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::InCombat, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::HasAmmo, true);
        action->SetDynamicTarget(DynamicTargetType::AmmoBox, myTeam == Team::Blue ? Team::Blue : Team::Red);
        action->SetActionPosition(blackboard->GetAmmoBoxPosition());
        action->SetExecutionRange(20.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<Heal>("Heal at Well", 1.0f);
        action->AddPrecondition(Conditions::LowHealth, { true, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::InCombat, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::LowHealth, false);
        action->SetDynamicTarget(DynamicTargetType::HealingWell, myTeam == Team::Blue ? Team::Blue : Team::Red);
        action->SetActionPosition(blackboard->GetHealingWellPosition());
        action->SetExecutionRange(20.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<FireAtTarget>("Fire at Leader", 1.0f);
        action->AddPrecondition(Conditions::HasAmmo, { true, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::LowHealth, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::AttackingLeader, true);
        action->SetDynamicTarget(DynamicTargetType::EnemyLeader, myTeam == Team::Blue ? Team::Blue : Team::Red);
        action->SetActionPosition(myTeam == Team::Blue ? blackboard->GetRedLeaderPos() : blackboard->GetBlueLeaderPos());
        action->SetExecutionRange(100.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<Revive>("Defend", 1.0f);
        action->AddPrecondition(Conditions::HasAmmo, { true, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::LowHealth, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::DefendingLeader, true);
        action->SetActionPosition(myTeam == Team::Blue ? blackboard->GetBlueLeaderPos() : blackboard->GetRedLeaderPos());
        action->SetExecutionRange(50.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<Retreat>("Retreat", 1.0f);
        action->AddPrecondition(Conditions::InCombat, { true, 1.0f, 1.0f });
        action->AddEffect(Conditions::InCombat, false);
        action->SetActionPosition({ 100.0f, 100.0f });
        action->SetExecutionRange(5.0f);
        myAvailableActions.push_back(action);
    }

    {
        auto action = std::make_shared<Revive>("Revive", 1.0f);
        action->AddPrecondition(Conditions::InCombat, { false, 1.0f, 1.0f });
        action->AddPrecondition(Conditions::LowHealth, { false, 1.0f, 1.0f });
        action->AddEffect(Conditions::LowSES, false);
        action->SetDynamicTarget(DynamicTargetType::HealingWell, myTeam == Team::Blue ? Team::Blue : Team::Red);
        action->SetActionPosition({ 0.0f, 0.0f });
        action->SetExecutionRange(10.0f);
        myAvailableActions.push_back(action);
    }

    //=== SPAWN SOLDIERS ===
    for (int i = 0; i < 4; ++i)
    {
        mySoldierSpawns[i] = (myTeam == Team::Blue) ? Tga::Vector2f{ 50.0f, 375.0f + (50.0f * i) } : Tga::Vector2f{ 1550.0f, 375.0f + (50.0f * i) };
        int id = blackboard->GetUniqueAgentID();
        auto soldier = std::make_shared<AIAgent>(id, myTeam, aNavmesh, mySoldierSpawns[i]);
        mySoldiers.push_back(soldier);
    }
    EventManager::GetInstance()->SendMessage(Message(eMessage::eRequestOrder, shared_from_this()));
}

void AISquad::SetPlan(const std::vector<std::shared_ptr<GOAPAction>>& aPlan)
{
    myPlan = aPlan;
    myCurrentActionIndex = 0;     
    for (auto& soldier : mySoldiers)
    {
        if (!myPlan.empty())
            soldier->SetCurrentAction(myPlan[myCurrentActionIndex]);
    }
}

void AISquad::SetOrder(std::unordered_map<GOAPGoal*, float> aGoalModifiers)
{
    const auto aggregatedState = GetAggregatedState();
    auto bestGoal = myPlanner->ChooseBestGoal(aggregatedState, myAvailableGoals, aGoalModifiers);

    std::cout << "Squad " << mySquadID << " picked goal: " << bestGoal->GetName() << "\n";

    const auto plan = myPlanner->PlanActions(aggregatedState, myAvailableActions, bestGoal);
    SetPlan(plan);
}

std::unordered_map<Conditions, ConditionData> AISquad::GetAggregatedState() const
{
    std::unordered_map<Conditions, ConditionData> aggregatedState;

    constexpr float MAX_AMMO = 10.0f;
    constexpr float MAX_HEALTH = 100.0f;
    constexpr float SES_PER_SOLDIER = 0.25f; //Squad Effectiveness Score (SES)
    constexpr float LOW_SES_THRESHOLD = 0.6f;
    constexpr float LOW_HEALTH_THRESHOLD = 0.3f;
    constexpr float AMMO_THRESHOLD = 0.2f;
    constexpr float COMBAT_RATIO_THRESHOLD = 0.5f;

    int aliveCount = 0;
    int inCombatCount = 0;
    int attackingLeaderCount = 0;
    int defendingLeaderCount = 0;

    float totalHealth = 0.0f;
    float totalAmmo = 0.0f;

    for (const auto& soldier : mySoldiers)
    {
        if (!soldier->IsAlive())
            continue;

        ++aliveCount;
        totalHealth += soldier->GetHealth();
        totalAmmo += static_cast<float>(soldier->GetAmmo());

        if (soldier->IsInCombat())
            ++inCombatCount;

        if (soldier->IsAttackingLeader())
            ++attackingLeaderCount;

        if (soldier->IsDefendingLeader())
            ++defendingLeaderCount;
    }

    const float maxHealth = aliveCount * MAX_HEALTH;
    const float maxAmmo = aliveCount * MAX_AMMO;
    const float squadSES = aliveCount * SES_PER_SOLDIER;

    bool isLowSES = squadSES < LOW_SES_THRESHOLD;
    bool isLowHealth = (maxHealth > 0.0f) && ((totalHealth / maxHealth) < LOW_HEALTH_THRESHOLD);
    bool hasAmmo = (maxAmmo > 0.0f) && ((totalAmmo / maxAmmo) > AMMO_THRESHOLD);
    bool isInCombat = (aliveCount > 0) && ((inCombatCount / static_cast<float>(aliveCount)) >= COMBAT_RATIO_THRESHOLD);
    bool isAttackingLeader = attackingLeaderCount >= 1;
    bool isDefendingLeader = defendingLeaderCount >= 1;

    aggregatedState[Conditions::LowHealth] = { isLowHealth, 0.0f, 0.0f };
    aggregatedState[Conditions::HasAmmo] = { hasAmmo,     0.0f, 0.0f };
    aggregatedState[Conditions::LowSES] = { isLowSES,    0.0f, 0.0f };
    aggregatedState[Conditions::InCombat] = { isInCombat,  0.0f, 0.0f };
    aggregatedState[Conditions::AttackingLeader] = { isAttackingLeader, 0.0f, 0.0f };
    aggregatedState[Conditions::DefendingLeader] = { isDefendingLeader, 0.0f, 0.0f };

    return aggregatedState;
}

float AISquad::GetSquadEffectiveness()
{
    int aliveCount = 0;
    for (const auto& soldier : mySoldiers)
    {
        if (soldier->IsAlive())
            ++aliveCount;
    }
    return std::min(1.0f, aliveCount * 0.25f); //0.25 per soldier
}

const std::vector<std::shared_ptr<AIAgent>>& AISquad::GetSoldiers() const
{
    return mySoldiers;
}

void AISquad::Update(float aDeltaTime)
{
    bool allActionsComplete = true;

    for (auto& soldier : mySoldiers)
    {
        soldier->Update(aDeltaTime);
        if (!soldier->HasCompletedAction())
            allActionsComplete = false;
    }

    DrawCurrentActionTarget();

    if (allActionsComplete)
        AdvancePlan();

    for (const auto& soldier : mySoldiers)
        soldier->Render();
}

void AISquad::DrawCurrentActionTarget()
{
    if (!myPlan.empty() && myCurrentActionIndex < myPlan.size())
    {
        const auto& engine = *Tga::Engine::GetInstance();
        Tga::DebugDrawer& debugDrawer = engine.GetDebugDrawer();
        Tga::Color color = (myTeam == Team::Blue) ? Tga::Color{ 0, 0, 1, 1 } : Tga::Color{ 1, 0, 0, 1 };

        debugDrawer.DrawCircle(myPlan[myCurrentActionIndex]->GetActionPosition(), 20.0f, color);
    }
}

void AISquad::AdvancePlan()
{
    if (!myPlan.empty() && myCurrentActionIndex < myPlan.size())
    {
        const std::string& prevActionName = myPlan[myCurrentActionIndex]->GetName();

        if (prevActionName == "Heal at Well")
            UpdateHealingWellLocation();

        else if (prevActionName == "Pick Ammo")
            UpdateAmmoBoxLocation();
    }

    myCurrentActionIndex++;

    if (myCurrentActionIndex < myPlan.size())
    {
        auto nextAction = myPlan[myCurrentActionIndex];
        for (auto& soldier : mySoldiers)
            soldier->SetCurrentAction(nextAction);
    }
    else
    {
        myPlan.clear();
        myCurrentActionIndex = 0;
        myWantsNewOrder = true;

        EventManager::GetInstance()->SendMessage(Message(eMessage::eRequestOrder, shared_from_this()));
        const auto& engine = *Tga::Engine::GetInstance();
        auto& debugDrawer = engine.GetDebugDrawer();

        for (const auto& action : myAvailableActions)
            debugDrawer.DrawCircle(action->GetActionPosition(), 20.0f, { 0, 0, 1, 1 });
    }
}

void AISquad::UpdateAmmoBoxLocation()
{
    auto* ammoAction = dynamic_cast<PickAmmo*>(myPlan[myCurrentActionIndex].get());
    if (!ammoAction) return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 2);

    int currentIndex = ammoAction->GetCurrentPosIndex();
    int newIndex = currentIndex;

    while (newIndex == currentIndex)
        newIndex = distr(gen);

    Blackboard::GetInstance()->SetAmmoBoxPosition(ammoAction->GetRandomPosition(newIndex));
}

void AISquad::UpdateHealingWellLocation()
{
    auto* healAction = dynamic_cast<Heal*>(myPlan[myCurrentActionIndex].get());
    if (!healAction) return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 2);

    int currentIndex = healAction->GetCurrentPosIndex();
    int newIndex = currentIndex;

    while (newIndex == currentIndex)
        newIndex = distr(gen);

    Blackboard::GetInstance()->SetHealingWellPosition(healAction->GetRandomPosition(newIndex));
}
