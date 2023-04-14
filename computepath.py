
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 20:53:51 2023

@author: gaohaotian
"""

from Astar import *


def compute_all_path(game_map_array,islive_worksations):
    
    obstacles_with_product=set()
    obstacles_without_product=set()
    workstations=[]
    for i in range(len(game_map_array)):
        for j in range(len(game_map_array)):
            if game_map_array[i][j]=='#':
                x=0.25 + j * 0.5
                y=(99.5 - i) * 0.5
                obstacle = Node(x, y)
                obstacles_with_product.add(obstacle)
                obstacles_without_product.add(obstacle)
                for xx in range(-3,4):
                    for yy in range(-3,4):
                        if (xx==0 and yy==0 or abs(xx*yy)>=6 ):
                            continue
                        if(abs(xx)<=2 and abs(yy)<=2):
                            obstacles_without_product.add(Node(x+xx/4,y+yy/4))
                        obstacles_with_product.add(Node(x+xx/4,y+yy/4))
                        
            if game_map_array[i][j]!='#' and game_map_array[i][j]!='.' and game_map_array[i][j]!='A' and game_map_array[i][j]!='\n':
                station=Node(0.25+(j)*0.5,(99.5-i)*0.5)
                workstation={
                    "type":int(game_map_array[i][j]),
                    "Node":station
                 }
                workstations.append(workstation)
                
                
    map_all_workstation_path=[]
    for i in range(len(workstations)):
        every_workstations_to_other=[]
        for j in range(len(workstations)):
            every_workstation_to_other={
                "path_of_noproduct": None,
                "path_of_product": None
            }
            every_workstations_to_other.append(every_workstation_to_other)
       
            
        map_all_workstation_path.append(every_workstations_to_other)      
    
    map_all_workstation_path=map_all_workstation_path
    
    for i in range(len(workstations)):
        if islive_worksations[i][0]==0:
            continue
        if workstations[i]["type"]==1 :
            # r=0.25+0.53
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                if workstations[j]["type"]==4 or workstations[j]["type"]==5 or workstations[j]["type"]==9:
                    path_of_product=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_with_product)
                    every_workstation_to_other={
                        "path_of_noproduct": None,
                        "path_of_product": path_of_product
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # 1
                
                
        elif workstations[i]["type"]==2 :
            # r=0.25+0.53
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                if workstations[j]["type"]==6 or workstations[j]["type"]==4 or workstations[j]["type"]==9:
                    path_of_product=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_with_product)
                    every_workstation_to_other={
                        "path_of_noproduct": None,
                        "path_of_product": path_of_product
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
                
        elif workstations[i]["type"]==3 :
            # r=0.25+0.53
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                if workstations[j]["type"]==5 or workstations[j]["type"]==6 or workstations[j]["type"]==9:
                    # path_of_noproduct=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles,0.25+0.45)
                    path_of_product=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_with_product)
                    every_workstation_to_other={
                        "path_of_noproduct": None,
                        "path_of_product": path_of_product
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
                
                
        elif workstations[i]["type"]==4 or workstations[i]["type"]==5 or workstations[i]["type"]==6 :
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                if workstations[j]["type"]!=8 :
                    path_of_noproduct=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_without_product)
                    path_of_product=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_with_product)
                    every_workstation_to_other={
                        "path_of_noproduct": path_of_noproduct,
                        "path_of_product": path_of_product
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
        
        elif workstations[i]["type"]==7:
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                # if workstations[j]["type"]!=8 :
                else:
                    path_of_noproduct=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_without_product)
                    path_of_product=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_with_product)
                    every_workstation_to_other={
                        "path_of_noproduct": path_of_noproduct,
                        "path_of_product": path_of_product
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
                
        elif workstations[i]["type"]==8 :
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                if workstations[j]["type"]!=9 and workstations[j]["type"]!=8 :
                    path_of_noproduct=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_without_product)
                    every_workstation_to_other={
                        "path_of_noproduct": path_of_noproduct,
                        "path_of_product": None
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
                    
        elif workstations[i]["type"]==9 :
            for j in range(len(workstations)):
                if i==j or islive_worksations[j][0]==0:
                    continue
                elif workstations[j]["type"]!=9 and workstations[j]["type"]!=8 :
                    path_of_noproduct=astar(workstations[i]["Node"], workstations[j]["Node"], obstacles_without_product)
                    every_workstation_to_other={
                        "path_of_noproduct": path_of_noproduct,
                        "path_of_product": None
                    }
                    map_all_workstation_path[i][j]=every_workstation_to_other
                    # if path_of_product!=None:
                    #     break
    return map_all_workstation_path