#include"../include/MyJson.h"
int MyJson::decodejson(string json_data){
    Json::Reader reader;
    Json::Value value;
    PlanPath_node one_node;
    try{
         //cout<<"222"<<endl;
         if(reader.parse(json_data,value)){
            if(value["action"].isNull()){
                    return ACT_NULL;
                }
                else {
                    //依据action进行json解析
                    action = value["action"].asString();
                    //1
                     if(action_.compare("move") == 0){
                        action_type = MOVE_ORD;
                        return RES_SUCC;
                     }
                    //2
                    if(action_.compare("deployRosParameter") == 0){
                        action_type = DeployRosParameter_ORD;
                        return RES_SUCC;
                    }
                    //3
                    if(action_.compare("patrolStartPathPlan") == 0){
                        action_type = PatrolStartPathPlan_ORD;
                        return RES_SUCC;
                    }
                    //4
                    if(action_.compare("patrolArriveNode") == 0){
                        action_type = PatrolArriveNode_ORD;
                        return RES_SUCC;
                    }
                    //5
                    if(action_.compare("stop") == 0){
                        action_type = STOP_ORD;
                        return RES_SUCC;
                    }
                    if(action_.compare("charge") == 0){
                        action_type = CHARGE_ORD;
                        return RES_SUCC;
                    }
                    
                }
            }
    }
    catch(exception &e){
        cout<<e.what()<<endl;
        return CATCH_ERROR;
    }
    return RES_SUCC;
    } 