import json
import uuid
import boto3
from decimal import Decimal
import uuid
from boto3.dynamodb.conditions import Key, Attr
from datetime import datetime
dynamodb_client = boto3.client('dynamodb', region_name="eu-west-1")
ddb = boto3.resource('dynamodb')
UserConfigTable = ddb.Table('UserConfig')
NotifyUserTable = ddb.Table('NotifyUser')

def lambda_handler(event, context):
    

    print(event)
    if event["Records"][0]["eventName"] =="MODIFY":
        newImage = event["Records"][0]["dynamodb"]["NewImage"]["op"]["M"]
        oldImage = event["Records"][0]["dynamodb"]["OldImage"]["op"]["M"]
        user_uuid =  event["Records"][0]["dynamodb"]["Keys"]["UserID"]["S"]
        already_mod = 0 
        opList=["AcOp","waterTapOp","lightsOp","boilerOp"]
        for i in opList:
            n_i =newImage[i]["M"]
            o_i = oldImage[i]["M"]
            
            if i != "AcOp":
                
                if n_i["errorCounter"]["N"] != o_i["errorCounter"]["N"] :
                    if Decimal(n_i["errorCounter"]["N"]) > 5:
                        already_mod = 1
                   
                        print("notify user")
                        topic= "set op."+ i +".errorCounter = :g ,op."+ i +".userRefused = :g,op."+ i +".userNotifyTotal = :g,"
                        notifyUserID = uuid.uuid4()
                        opName = i
                        if i == "boilerOp":
                            item = Decimal(n_i["averageTemperatureGap"]["N"] ) + Decimal( n_i["waterOptimalTemperature"]["N"])
                            
                            #clean fix
                            topic +="op." +i +".averageTemperatureGap =:g "
                            print(i,topic)
                            response3 = UserConfigTable.update_item(Key ={'UserID' :  user_uuid},
                            UpdateExpression=topic ,
                            ExpressionAttributeValues={':g':0 },
                            ReturnValues="UPDATED_NEW") 
                            
                            #notify User
                            msgToUser="We have seen that there is a difference between the water temperature you entered as optimal and the water temperature at which you turned off the boiler recently, we suggest you change the optimal water temperature to "
                            msgToUser+= str('%0.*f' % (isinstance(item, float) , item))
                            print("insert item to NotifyUser")
                            response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                            
                            
                        else:
                            item = Decimal(n_i["averageTimeError"]["N"] ) + Decimal( n_i["emptyRoomTime"]["N"])
                            
                            
                            
                            #clean fix
                            topic +="op." +i +".averageTimeError =:g "
                            print(i,topic)
                            response3 = UserConfigTable.update_item(Key ={'UserID' : user_uuid},
                            UpdateExpression=topic ,
                            ExpressionAttributeValues={':g':0 },
                            ReturnValues="UPDATED_NEW") 
                            
                            #notify User
                            device_op = "lights on" 
                            if i == "waterTapOp":
                                device_op = "water tap is running" 
                            
                                
                            msgToUser="We have seen that there is a difference between the empty room time you entered as optimal and the time you usually leave the room empty when the "+ device_op +", we suggest you change the optimal empty room time when the "+ device_op +" to "
                            msgToUser+= str('%0.*f' % (isinstance(item, float) , item))
                            print("insert item to NotifyUser")
                            response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                            
                if already_mod==0:
           
                    print(" - " * 10)
                           
                    if n_i["userRefused"]["N"] != o_i["userRefused"]["N"]:
                        if Decimal(n_i["userRefused"]["N"]) > 5:
                            if Decimal(int(n_i["userRefused"]["N"])/int(n_i["userNotifyTotal"]["N"])) > 0.5:
                                topic= "set op."+ i +".errorCounter = :g ,op."+ i +".userRefused = :g,op."+ i +".userNotifyTotal = :g,"
                                
                                if i == "boilerOp":
                                    item = Decimal(n_i["averageTemperatureGap"]["N"]) + Decimal(n_i["waterOptimalTemperature"]["N"]) + 1
                                    topic ="Set op." +i +"averageTemperatureGap =:z,op." +i +"waterOptimalTemperature =:i  , op." +i +"ourLearn.waterOptimalTemperature =:i ,op." +i +".errorCounter = :z,op." +i +".userRefused =:z,op." +i +".userNotifyTotal =:z "
                                    print(i,topic)
                                    #clean fix
                                    topic +="op." +i +".averageTemperatureGap =:g "
                                    print(i,topic)
                                    response3 = UserConfigTable.update_item(Key ={'UserID' : user_uuid},
                                    UpdateExpression=topic ,
                                    ExpressionAttributeValues={':g':0 },
                                    ReturnValues="UPDATED_NEW") 
                                    
                                    #notify User
                                    msgToUser="We have seen that you have many times refused our recommendation to turn off the boiler when it reaches the optimum water temperature you have set for the system, we recommend changing the optimum water temperature to your preference"
                                    print("insert item to NotifyUser")
                                    response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                                    
                                    
                                    
                                else:
                                    item = Decimal(n_i["averageTimeError"]["N"] )+ Decimal(n_i["emptyRoomTime"]["N"]) + 1
                                    topic ="Set op." +i +"averageTimeError =:z,op." +i +"emptyRoomTime =:i  , op." +i +"ourLearn.emptyRoomTime =:i ,op." +i +".errorCounter = :z,op." +i +".userRefused =:z,op." +i +".userNotifyTotal =:z "
                                    print(i,topic)
                                    #clean fix
                                    topic +="op." +i +".averageTimeError =:g "
                                    print(i,topic)
                                    response3 = UserConfigTable.update_item(Key ={'UserID' : user_uuid},
                                    UpdateExpression=topic ,
                                    ExpressionAttributeValues={':g':0 },
                                    ReturnValues="UPDATED_NEW") 
                                    
                                    device_op = "lights" 
                                    if i == "waterTapOp":
                                        device_op = "water tap" 
                                    
                                    #notify User
                                    msgToUser="We have seen that you have many times refused our recommendation to turn off the "+ device_op +" when the optimal empty room time you set for the system passes, we recommend changing the optimum empty room time to your preference"
                                    print("insert item to NotifyUser")
                                    response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                                   

            else:
                empty_flag = 0 
                temp_flag = 0
                refused_empty =0 
                refused_temp =0 
                temp = Decimal(n_i["averageTemperatureGap"]["N"] )+Decimal(n_i["roomOptimalTemperature"]["N"] )
                empty = Decimal(n_i["averageTimeError"]["N"] )+Decimal(n_i["emptyRoomTime"]["N"] )
                if n_i["TimeErrorCounter"]["N"] != o_i["TimeErrorCounter"]["N"]:
                    if Decimal(n_i["TimeErrorCounter"]["N"]) > 5:
                        
                        empty_flag=1
                if n_i["tempetureGapErrorCounter"]["N"] != o_i["tempetureGapErrorCounter"]["N"]:
                    if Decimal(n_i["tempetureGapErrorCounter"]["N"]) > 5:
                        temp_flag =1
                        
                if n_i["userRefusedToChangeTemperature"]["N"] != o_i["userRefusedToChangeTemperature"]["N"]:
                    if Decimal(n_i["userRefusedToChangeTemperature"]["N"]) > 5:
                        if Decimal(int(n_i["userRefusedToChangeTemperature"]["N"])/int(n_i["userNotifyToChangeTemperature"]["N"])) > 0.5:
                            
                            temp +=1
                            temp_flag =1
                            refused_temp =1
                if n_i["userRefusedToTurnOff"]["N"] != o_i["userRefusedToTurnOff"]["N"]:
                    if Decimal(n_i["userRefusedToTurnOff"]["N"]) > 5:
                        if Decimal(int(n_i["userRefusedToTurnOff"]["N"])/int(n_i["userNotifyToTurnOff"]["N"])) > 0.5:
                            
                            empty+=2
                            empty_flag=1
                            refused_empty =1
                            
             
                #neww##
                
                if temp_flag==1:
                    topic= "set op."+ i +".userNotifyToChangeTemperature = :g ,op."+ i +".userRefusedToChangeTemperature = :g,op."+ i +".tempetureGapErrorCounter = :g,op."+ i +".averageTemperatureGap = :g"
                    item = temp
                    print("notify user")
                    notifyUserID = uuid.uuid4()
                    opName = i
                    op2 ="empty"
              
                    #clean fix
                    print(i,topic)
                    response3 = UserConfigTable.update_item(Key ={'UserID' :  user_uuid},
                    UpdateExpression=topic ,
                    ExpressionAttributeValues={':g':0 },
                    ReturnValues="UPDATED_NEW") 
                    
                    #notify User
                    if refused_temp == 1 : 
                        msgToUser="We have seen that you have often refused our recommendation to change the air conditioner temperature when it reaches the optimum temperature you have set for the system, we recommend changing the optimum temperature to your preference"
                    else:    
                        msgToUser="We have seen that there is a difference between the air conditioner temperature you have entered as optimal and the air conditioner temperature that you have recently adjusted to, we suggest you change the optimal air conditioner temperature to "
                        msgToUser+= str('%0.*f' % (isinstance(item, float) , item))
                    print("insert item to NotifyUser")
                    response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                   
    
                if empty_flag ==1:
                    topic= "set op."+ i +".userNotifyToTurnOff = :g ,op."+ i +".userRefusedToTurnOff = :g,op."+ i +".TimeErrorCounter = :g,op."+ i +".averageTimeError = :g"
                    item = empty
                    print("notify user")
                    notifyUserID = uuid.uuid4()
                    opName = i
                    op2 ="temp"
                    
                    #clean fix
                    print(i,topic)
                    response3 = UserConfigTable.update_item(Key ={'UserID' :  user_uuid},
                    UpdateExpression=topic ,
                    ExpressionAttributeValues={':g':0 },
                    ReturnValues="UPDATED_NEW") 
                    
                    #notify User
                    if refused_empty == 1 : 
                        msgToUser="We have seen that you have many times refused our recommendation to turn off the air conditioner when the optimal empty room time you set for the system passes, we recommend changing the optimal empty room time to your preference"
                    else:
                        msgToUser="We have seen that there is a difference between the empty room time you entered as optimal and the time you usually leave the room empty when the air conditioner is running, we suggest you change the optimal empty room time when the air conditioner is running to "
                        msgToUser+= str('%0.*f' % (isinstance(item, float) , item))
                    print("insert item to NotifyUser")
                    response2 = NotifyUserTable.put_item(Item={"floor": "","NotifyUserID": str(notifyUserID), "room":"","userAnswer": 0,"UserID": user_uuid ,"op": i,"msgToUser": msgToUser})
                   
                   
        
            print(" - " * 20)
                
        
        

    return {
        'statusCode': 200,
        'body': json.dumps('Hello from Lambda!')
    }
