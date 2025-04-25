#include "AILeader.h"
#include "AISquad.h"
#include "Blackboard.h"
#include "EventManager.h"
#include <imgui\imgui.h>
#include "AIAgent.h"
#include "navmesh.h"
#include <string>

#undef SendMessage

void AILeader::Init(Team aTeam, std::shared_ptr<Navmesh> aNavmesh)
{
    EventManager* eventManager = EventManager::GetInstance();
    Blackboard* blackboard = Blackboard::GetInstance();

    eventManager->Subscribe(eMessage::eRequestOrder, this);
    eventManager->Subscribe(eMessage::eWorldStateChanged, this);

    myTeam = aTeam;

    const int squadAmount = 1;
    for (int i = 0; i < squadAmount; i++)
    {
        std::shared_ptr<AISquad> squad = std::make_shared<AISquad>(blackboard->GetUniqueSquadID());
        squad->Init(aTeam, aNavmesh);
        mySquads.push_back(squad);
        blackboard->RegisterSquad(squad);
    }

    Tga::Color textColor;
    Tga::Vector2f textPos;
    if (myTeam == Team::Blue)
    {
        myPosition = blackboard->GetBlueLeaderPos();
        textPos = blackboard->GetBlueLeaderPos() + Tga::Vector2f{ -100.0f, 35.0f };
        textColor = { 0,0,1,1 };
    }      
    else
    {
        myPosition = blackboard->GetRedLeaderPos();
        textPos = blackboard->GetRedLeaderPos() + Tga::Vector2f{ 0.0f, 35.0f };
        textColor = { 1,0,0,1 };
    }
        
    myHealthText = Tga::Text{ L"Text/arial.ttf", Tga::FontSize_18 };
    myHealthText.SetColor(textColor);
    myHealthText.SetPosition(textPos);
}

void AILeader::Update(const float aDeltaTime)
{   
    if (myHealth <= 0)
    {
        myIsAlive = false;
    }

    Blackboard* blackboard = Blackboard::GetInstance();
    blackboard->UpdateWorldStates();

    for (auto squad : mySquads)
    {
        squad->Update(aDeltaTime);
    }  
#ifdef _DEBUG
    ImGui::Begin("AI Leader Debug");

    //GLOBAL WORLD STATES
    ImGui::Text("Global Blackboard States:");
    bool redLeaderAlive = blackboard->GetGlobalState(WorldState::IsRedLeaderAttacked);
    bool blueLeaderAlive = blackboard->GetGlobalState(WorldState::IsBlueLeaderAttacked);
    bool healingAvailable = blackboard->GetGlobalState(WorldState::IsHealingClose);
    bool ammoAvailable = blackboard->GetGlobalState(WorldState::IsAmmoClose);

    if (ImGui::Checkbox("Red Leader Attacked", &redLeaderAlive))
        blackboard->SetGlobalState(WorldState::IsRedLeaderAttacked, redLeaderAlive);
    if (ImGui::Checkbox("Blue Leader Attacked", &blueLeaderAlive))
        blackboard->SetGlobalState(WorldState::IsBlueLeaderAttacked, blueLeaderAlive);
    if (ImGui::Checkbox("Healing Available", &healingAvailable))
        blackboard->SetGlobalState(WorldState::IsHealingClose, healingAvailable);
    if (ImGui::Checkbox("Ammo Available", &ammoAvailable))
        blackboard->SetGlobalState(WorldState::IsAmmoClose, ammoAvailable);

    ImGui::Separator();

    // SQUAD + AGENT DEBUG
    for (size_t i = 0; i < mySquads.size(); ++i)
    {
        auto& squad = mySquads[i];

        std::string header = "Squad " + std::to_string(squad->GetID());
        if (ImGui::CollapsingHeader(header.c_str()))
        {
            // Show agent stats
            const auto& agents = squad->GetSoldiers();
            for (size_t j = 0; j < agents.size(); ++j)
            {
                auto& agent = agents[j];
                std::string label = "Agent " + std::to_string(i) + "_" + std::to_string(j); // i = squad index
                if (ImGui::TreeNode(label.c_str()))
                {
                    float health = agent->GetHealth();
                    int ammo = agent->GetAmmo();
                    bool alive = agent->IsAlive();
                    bool inCombat = agent->IsInCombat();

                    std::string healthID = "Health##" + std::to_string(i) + "_" + std::to_string(j);
                    std::string ammoID = "Ammo##" + std::to_string(i) + "_" + std::to_string(j);
                    std::string aliveID = "Alive##" + std::to_string(i) + "_" + std::to_string(j);
                    std::string combatID = "In Combat##" + std::to_string(i) + "_" + std::to_string(j);

                    if (ImGui::DragFloat(healthID.c_str(), &health, 1.0f, 0.0f, 100.0f))
                        agent->SetHealth(health);
                    if (ImGui::DragInt(ammoID.c_str(), &ammo, 1, 0, 10))
                        agent->SetAmmo(ammo);
                    if (ImGui::Checkbox(aliveID.c_str(), &alive))
                        agent->SetIsAlive(alive);
                    if (ImGui::Checkbox(combatID.c_str(), &inCombat))
                        agent->UpdateCombatData(inCombat, false, false);

                    ImGui::TreePop();
                }
            }

            //SEND NEW ORDER
            if (squad->WantsNewOrder())
            {
                std::string btnLabel = "Send New Order##" + std::to_string(i);
                if (ImGui::Button(btnLabel.c_str()))
                {
                    EventManager::GetInstance()->SendMessage(Message(eMessage::eRequestOrder, squad));
                    squad->ClearOrderRequest();
                }
            }
        }
    }

    ImGui::End();
#endif // !_DEBUG  
    myHealthText.SetText("HP: " + std::to_string(myHealth));
    myHealthText.Render();
}

void AILeader::Render(){}

void AILeader::TakeDamage(const float aDamage)
{
    myHealth -= (int)aDamage;
}

std::unordered_map<GOAPGoal*, float> AILeader::EvaluateGoalModifiers(const std::shared_ptr<AISquad>& aSquad)
{
    std::unordered_map<GOAPGoal*, float> goalModifiers;
    Blackboard* blackboard = Blackboard::GetInstance();
    const Team team = aSquad->GetTeam();
    const int squadID = aSquad->GetID();

    // World state
    const bool isRedLeaderUnderAttack = blackboard->GetGlobalState(WorldState::IsRedLeaderAttacked);
    const bool isBlueLeaderUnderAttack = blackboard->GetGlobalState(WorldState::IsBlueLeaderAttacked);
    const bool healingAvailable = blackboard->GetGlobalState(WorldState::IsHealingClose);
    const bool ammoAvailable = blackboard->GetGlobalState(WorldState::IsAmmoClose);
    const bool squadsAreNearby = blackboard->GetGlobalState(WorldState::SquadsAreNearby);

    const float squadSES = aSquad->GetSquadEffectiveness();
    const float enemySES = blackboard->GetEnemySquadSES(squadID, team); // <- TODO: remove hardcoded 0.75f in original
    const bool hasAdvantage = squadSES >= enemySES;

    const bool shouldAttackLeader = (team == Team::Blue) ? !isRedLeaderUnderAttack : !isBlueLeaderUnderAttack;
    const bool shouldDefendLeader = (team == Team::Blue) ? isBlueLeaderUnderAttack : isRedLeaderUnderAttack;

    //Score each goal with contextual modifiers
    for (const auto& goal : aSquad->GetAvailableGoals())
    {
        const std::string& name = goal->GetName();
        float modifier = 0.0f;

        if (name == "Attack Enemy Leader")
        {
            modifier += shouldAttackLeader ? 0.5f : -0.5f;
            modifier += squadsAreNearby ? 0.0f : -0.3f;
            modifier += ammoAvailable ? 0.2f : 0.0f;
        }
        else if (name == "Attack Enemy Squad")
        {
            modifier += hasAdvantage ? 0.6f : -0.8f;
            modifier += squadsAreNearby ? 0.4f : 0.0f;
            modifier += ammoAvailable ? 0.2f : 0.0f;
        }
        else if (name == "Defend Friendly Leader")
        {
            modifier += shouldDefendLeader ? 1.5f : 0.0f;
        }       
        else if (name == "Keep Squad Alive")
        {
            modifier += hasAdvantage ? 0.0f : 1.5f;
            modifier += healingAvailable ? 0.4f : -0.2f;
        }

        goalModifiers[goal.get()] = modifier;
    }
    return goalModifiers;
}

void AILeader::Recieve(const Message& aMsg)
{
    if (aMsg.GetMessageType() == eMessage::eRequestOrder)
    {
        std::shared_ptr<AISquad> squad = aMsg.GetData<std::shared_ptr<AISquad>>();
        auto goalModifiers = EvaluateGoalModifiers(squad);
        squad->SetOrder(goalModifiers);
    }
    else if (aMsg.GetMessageType() == eMessage::eWorldStateChanged)
    {
        for (auto& squad : mySquads)
        {
            auto goalModifiers = EvaluateGoalModifiers(squad);
            squad->SetOrder(goalModifiers);
        }
    }
}
