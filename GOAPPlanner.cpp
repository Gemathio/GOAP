#include "GOAPPlanner.h"
#include <iostream>
#include <algorithm>

struct SearchNode
{
    std::shared_ptr<GOAPAction> action;
    std::unordered_map<Conditions, ConditionData> state;
    float cost;
    std::shared_ptr<SearchNode> parent;
};

struct CompareSearchNode
{
    bool operator()(const std::shared_ptr<SearchNode>& aNode0, const std::shared_ptr<SearchNode>& aNode1) const
    {
        return aNode0->cost > aNode1->cost;
    }
};

std::vector<std::shared_ptr<GOAPAction>> GOAPPlanner::PlanActions(
    const std::unordered_map<Conditions, ConditionData>& aAIState,
    const std::vector<std::shared_ptr<GOAPAction>>& aActions,
    const std::shared_ptr<GOAPGoal>& aGoal)
{
    std::priority_queue<std::shared_ptr<SearchNode>, std::vector<std::shared_ptr<SearchNode>>, CompareSearchNode> openSet;

    auto startNode = std::make_shared<SearchNode>();
    startNode->state = aAIState;
    startNode->cost = 0.0f;
    openSet.push(startNode);

    while (!openSet.empty())
    {
        auto currentNode = openSet.top();
        openSet.pop();

        //Check if current node satisfies the goal
        bool goalSatisfied = true;
        for (const auto& goalCondition : aGoal->GetConditions())
        {
            auto it = currentNode->state.find(goalCondition.first);
            if (it == currentNode->state.end() || it->second.state != goalCondition.second.state)
            {
                goalSatisfied = false;
                break;
            }
        }

        if (goalSatisfied)
        {
            std::vector<std::shared_ptr<GOAPAction>> plan;
            auto node = currentNode;
            while (node->parent != nullptr)
            {
                plan.push_back(node->action);
                node = node->parent;
            }
            std::reverse(plan.begin(), plan.end());
            return plan;
        }

        //Expand current node with all valid actions
        for (const auto& action : aActions)
        {
            if (!action->IsNodeValid(currentNode->state))
                continue;

            auto newState = currentNode->state;
            action->ApplyEffects(newState);

            bool stateChanged = false;
            for (const auto& effect : action->GetEffects())
            {
                if (currentNode->state.find(effect.first) == currentNode->state.end() ||
                    currentNode->state.at(effect.first).state != newState.at(effect.first).state)
                {
                    stateChanged = true;
                    break;
                }
            }

            if (!stateChanged)
                continue;

            auto childNode = std::make_shared<SearchNode>();
            childNode->action = action;
            childNode->state = newState;
            childNode->cost = currentNode->cost + action->GetCost();
            childNode->parent = currentNode;

            openSet.push(childNode);
        }
    }
    return {}; //No valid sequence found
}

float GOAPPlanner::CalculateHeuristic(const std::unordered_map<Conditions, ConditionData>& aCurrentState, const std::unordered_map<Conditions, ConditionData>& aGoalConditions)
{
    float hCost = 0.0f;

    for (const auto& condition : aGoalConditions)
    {
        if (aCurrentState.find(condition.first) == aCurrentState.end())
        {
            hCost += 1.0f;
        }
    }
    return hCost;
}

bool GOAPPlanner::IsActionRelevantToGoal(const std::shared_ptr<GOAPAction>& aAction, const std::shared_ptr<GOAPGoal>& aGoal)
{
    for (const auto& effect : aAction->GetEffects())
    {
        if (aGoal->HasCondition(effect.first))
        {
            return true;
        }
    }
    return false;
}

std::shared_ptr<GOAPGoal> GOAPPlanner::ChooseBestGoal(const std::unordered_map<Conditions, ConditionData>& aAIState, 
    const std::vector<std::shared_ptr<GOAPGoal>>& aAvailableGoals,
    const std::unordered_map<GOAPGoal*, float>& aGoalModifiers)
{
    float highestPriority = -std::numeric_limits<float>::infinity();
    std::shared_ptr<GOAPGoal> bestGoal = nullptr;

    for (const auto& goal : aAvailableGoals)
    {
        //Skip goals already satisfied
        bool alreadySatisfied = true;
        for (const auto& condition : goal->GetConditions())
        {
            auto it = aAIState.find(condition.first);
            if (it == aAIState.end() || it->second.state != condition.second.state)
            {
                alreadySatisfied = false;
                break;
            }
        }
        if (alreadySatisfied)
            continue;

        float dynamicPriority = goal->EvaluatePriority(aAIState);

        auto modifierIt = aGoalModifiers.find(goal.get());
        if (modifierIt != aGoalModifiers.end())
        {
            dynamicPriority += modifierIt->second;
        }

        if (dynamicPriority > highestPriority)
        {
            highestPriority = dynamicPriority;
            bestGoal = goal;
        }
    }
    return bestGoal;
}
