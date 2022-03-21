
#include <despot/util/mongoDB_Bridge.h>
#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <sstream>
#include <unistd.h>

#include <nlohmann/json.hpp> 
using json = nlohmann::json; 

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::make_document;
using bsoncxx::builder::basic::make_array;

namespace despot {
  bool MongoDB_Bridge::isInit = false;
  mongocxx::client MongoDB_Bridge::client;
  mongocxx::instance MongoDB_Bridge::instance;
  mongocxx::database MongoDB_Bridge::db;
  mongocxx::collection MongoDB_Bridge::actionToExecuteCollection;
  mongocxx::collection MongoDB_Bridge::moduleResponseColllection;
  mongocxx::collection MongoDB_Bridge::localVariableColllection;
  mongocxx::collection MongoDB_Bridge::actionsCollection;
  mongocxx::collection MongoDB_Bridge::globalVariablesAssignmentsColllection;
  mongocxx::collection MongoDB_Bridge::SolversCollection;

  mongocxx::collection MongoDB_Bridge::beliefStatesColllection;
  int MongoDB_Bridge::currentActionSequenceId = 0;
  std::chrono::milliseconds MongoDB_Bridge::firstSolverIsAliveDateTime = std::chrono::milliseconds(0);

  void MongoDB_Bridge::Init()
  {
    if (!MongoDB_Bridge::isInit)
    {
      MongoDB_Bridge::isInit = true;
       
       // MongoDB_Bridge::instance = mongocxx::instance{};
      mongocxx::uri uri("mongodb://localhost:27017");
      MongoDB_Bridge::client = mongocxx::client(uri);

      MongoDB_Bridge::db = MongoDB_Bridge::client["AOS"];
      MongoDB_Bridge::actionToExecuteCollection = MongoDB_Bridge::db["ActionsForExecution"];
      MongoDB_Bridge::moduleResponseColllection = MongoDB_Bridge::db["ModuleResponses"];
      MongoDB_Bridge::globalVariablesAssignmentsColllection = MongoDB_Bridge::db["GlobalVariablesAssignments"];
      MongoDB_Bridge::localVariableColllection = MongoDB_Bridge::db["localVariables"];
      MongoDB_Bridge::actionsCollection = MongoDB_Bridge::db["Actions"];
      MongoDB_Bridge::SolversCollection = MongoDB_Bridge::db["Solvers"];
      MongoDB_Bridge::beliefStatesColllection = MongoDB_Bridge::db["BeliefStates"];
    }
}

// void MongoDB_Bridge::UpdateActionResponse(std::string actionName, std::string actionResponse)
// {
//   auto filter = document{} << "wasRead" << false << "Module" << actionName << finalize;
//   //auto update = document{} << "$set" << open_document << "wasRead" << true << "moduleResponseText" << actionResponse << close_document << finalize;
//   std::stringstream ss;
//   ss<< "{\"$set\" : {\"wasRead\":true, \"moduleResponseText\" : \"" << actionResponse << "\"}}"; 
  
//   MongoDB_Bridge::moduleResponseColllection.update_one(filter.view(), bsoncxx::from_json(ss.str()));
// }
void MongoDB_Bridge::GetSolverDetails(bool& shutDown, bool& isFirst, int solverId)
{
  MongoDB_Bridge::Init();
  auto filter = document{} << "SolverId" << solverId << finalize;
  bool found = false;
 
    mongocxx::cursor cursor = MongoDB_Bridge::SolversCollection.find({filter});
    for(auto doc : cursor) 
    {
      found = true;
      std::string s = bsoncxx::to_json(doc);
      json jsonObj = json::parse(s);
     
     shutDown = !jsonObj["ServerShutDownRequestDateTime"].is_null();
     isFirst = jsonObj["FirstSolverIsAliveDateTime"].is_null();

     //if server is alive date is initialized but not as in DB. then this solver is a zombie process and should by ShutDown
     shutDown |=  !(
                    (isFirst && MongoDB_Bridge::firstSolverIsAliveDateTime == std::chrono::milliseconds(0)) || 
                    (!isFirst && doc["FirstSolverIsAliveDateTime"].get_date().value == MongoDB_Bridge::firstSolverIsAliveDateTime));
    }
    if(!found)
    {
      isFirst = false;
      shutDown = true;
    }
}

void MongoDB_Bridge::UpdateSolverDetails(bool isFirst, int solverId)
{
  MongoDB_Bridge::Init();
  
  auto now = std::chrono::system_clock::now();
  auto builder = bsoncxx::builder::stream::document{};

  bsoncxx::document::value filter = builder << "SolverId" << solverId << finalize;
  bsoncxx::document::value update = isFirst ? builder << "$set" << open_document << "FirstSolverIsAliveDateTime" << bsoncxx::types::b_date(now) << "SolverIsAliveDateTime" << bsoncxx::types::b_date(now) << close_document << finalize
                                            : builder << "$set" << open_document << "SolverIsAliveDateTime" << bsoncxx::types::b_date(now) << close_document << finalize;

if(isFirst)
{
  MongoDB_Bridge::firstSolverIsAliveDateTime = bsoncxx::types::b_date(now).value; //set this solver first is alive date
}
  MongoDB_Bridge::SolversCollection.update_one(filter.view(), update.view()); 
}

void MongoDB_Bridge::SaveInternalActionResponse(std::string actionName, bsoncxx::oid actionForExecuteId, std::string observationText)
{

 MongoDB_Bridge::Init(); 
 auto now = std::chrono::system_clock::now();
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value =
          builder << "ActionSequenceId" << MongoDB_Bridge::currentActionSequenceId
                  << "Module" << actionName
                  << "ModuleResponseText" << observationText
                  << "StartTime" << bsoncxx::types::b_date(now) 
                  << "ActionForExecutionId" << actionForExecuteId
                  << "EndTime" << bsoncxx::types::b_date(now)
                  << finalize;
 
  MongoDB_Bridge::moduleResponseColllection.insert_one(doc_value.view());
}

std::map<std::string, bool> MongoDB_Bridge::WaitForActionResponse(bsoncxx::oid actionForExecuteId, std::string& actionTextObservation)
{

  std::map<std::string, bool> globalVarUpdates;
  std::vector<bsoncxx::document::view> moduleLocalVars;
  MongoDB_Bridge::Init();
  auto filter = document{} << "ActionForExecutionId" << actionForExecuteId << finalize;
  bool actionFinished = false;

  while (!actionFinished)
  {
    mongocxx::cursor cursor = MongoDB_Bridge::moduleResponseColllection.find({filter});
    for(auto doc : cursor) 
    {
      actionFinished = true;
      actionTextObservation = doc["ModuleResponseText"].get_utf8().value.to_string(); 
    }
    if(actionFinished)
    {
      usleep(500000);
        auto filter2 = document{} << "UpdatingActionSequenceId" << MongoDB_Bridge::currentActionSequenceId << finalize;
        mongocxx::cursor cursor2 = MongoDB_Bridge::globalVariablesAssignmentsColllection.find({filter2});
        for(auto doc : cursor2) 
        {
          std::string globalVariableName = doc["GlobalVariableName"].get_utf8().value.to_string();
          bool isInit = doc["IsInitialized"].get_bool();
          globalVarUpdates[globalVariableName] = isInit;
        }
        break;
    }
  }

  
  return globalVarUpdates;
}

bsoncxx::oid MongoDB_Bridge::SendActionToExecution(int actionId, std::string actionName, std::string actionParameters)
{
  MongoDB_Bridge::Init(); 
  auto now = std::chrono::system_clock::now();
  auto builder = bsoncxx::builder::stream::document{}; 
  bsoncxx::document::value doc_value = !actionParameters.empty() ? ( 
                                                                builder << "ActionID" << actionId 
                                                                               << "ActionName" << actionName
                                                                               << "ActionSequenceId" << MongoDB_Bridge::currentActionSequenceId
                                                                               << "RequestCreateTime" << bsoncxx::types::b_date(now)
                                                                               << "Parameters" <<  bsoncxx::from_json(actionParameters)
                                                                               << finalize)
                                                                 : (
                                                                       builder << "ActionID" << actionId 
                                                                               << "ActionName" << actionName
                                                                               << "ActionSequenceId" << MongoDB_Bridge::currentActionSequenceId
                                                                               << "RequestCreateTime" << bsoncxx::types::b_date(now)
                                                                                << "Parameters" << open_document 
                                                                                << close_document << finalize);

  auto retVal =MongoDB_Bridge::actionToExecuteCollection.insert_one(doc_value.view());

  bsoncxx::oid oid = retVal->inserted_id().get_oid().value;
  return oid;
}

std::string MongoDB_Bridge::SampleFromBeliefState(int skipStates, int takeStates)
{
  MongoDB_Bridge::Init(); 
  mongocxx::options::find opts{};
  opts.projection(make_document(kvp("BeliefeState", make_document(kvp("$slice",make_array(skipStates, takeStates)))))); 
  bool found = false;

  auto doc = MongoDB_Bridge::beliefStatesColllection.find_one(make_document(kvp("ActionSequnceId", -1)), opts);
  return bsoncxx::to_json(doc.value());

}

void MongoDB_Bridge::SaveBeliefState(std::string currentActionBelief, std::string currentBelief)
{
  MongoDB_Bridge::Init(); 
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = bsoncxx::from_json(currentActionBelief); 
                                                    
  MongoDB_Bridge::beliefStatesColllection.insert_one(doc_value.view());


  auto filter = document{} << "ActionSequnceId" << -1 << finalize;
  mongocxx::options::replace option;
  option.upsert(true);
  bsoncxx::document::value currentBeliefDocValue = bsoncxx::from_json(currentBelief); 
  MongoDB_Bridge::beliefStatesColllection.replace_one(filter.view(), currentBeliefDocValue.view(), option);
}

void MongoDB_Bridge::RegisterAction(int actionId, std::string actionName, std::string actionParameters, std::string actionDescription)
{
  MongoDB_Bridge::Init(); 
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = !actionParameters.empty() ? (
                                                                       builder << "ActionID" << actionId 
                                                                               << "ActionName" << actionName
                                                                               << "ActionDecription" << actionDescription
                                                                               << "ActionConstantParameters" << open_array
                                                                               << [&](bsoncxx::builder::stream::array_context<> arr)
                                                                       { arr << bsoncxx::from_json(actionParameters); }
                                                                               << close_array << finalize)
                                                                 : (
                                                                       builder << "ActionID" << actionId 
                                                                               << "ActionName" << actionName
                                                                               << "ActionDecription" << actionDescription
                                                                               << "ActionConstantParameters" << open_array
                                                                               << close_array << finalize);
auto filter = document{} << "ActionID" << actionId << finalize;
  mongocxx::options::replace option;
  option.upsert(true);
  MongoDB_Bridge::actionsCollection.replace_one(filter.view(), doc_value.view(), option);
}
} 

//Generate document with array dynamically
/*
auto now = std::chrono::system_clock::now();
  auto builder = bsoncxx::builder::stream::document{};
  bsoncxx::document::value doc_value = builder << "actionName" << actionName
  << "RequestCreateTime" << bsoncxx::types::b_date(now)
  << "wasHandled" << false


  
  << "HandleTime" << bsoncxx::types::b_date(now - std::chrono::hours(1)) 
  << "parameters" << open_array
<< [&](bsoncxx::builder::stream::array_context<> arr) {
        for (int i = 0; i < parameterValues.size(); i++)
        {
            arr << open_document << parameterNames[i] << parameterValues[i] << close_document;
        }
    } << close_array << finalize;
*/