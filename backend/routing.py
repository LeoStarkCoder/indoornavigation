import heapq
from map_data import NODES, EDGES

def shortest_path(start, goal, blocked_labels=frozenset()):
    graph={n:[] for n in NODES}
    for e in EDGES:
        if blocked_labels.intersection(e.get("obstacle_classes",[])): continue
        a,b=e["from"],e["to"]; w=float(e["distance"])
        graph[a].append((b,w)); graph[b].append((a,w))
    queue=[(0.0,start,[])]; best={}
    while queue:
        cost,node,path=heapq.heappop(queue)
        if node in best and best[node]<=cost: continue
        best[node]=cost; path=path+[node]
        if node==goal: return path,cost
        for neighbor,weight in graph[node]:
            if neighbor not in best: heapq.heappush(queue,(cost+weight,neighbor,path))
    return [],float("inf")

def directions_for(path, blocked_labels=frozenset()):
    result=[]
    for i,node in enumerate(path):
        name=NODES[node]["name"]
        if i==0: text=f"Start at {name}."
        elif i==len(path)-1: text=f"You have arrived at {name}."
        else: text=f"Continue to {name}."
        result.append({"step":i+1,"node":node,"text":text})
    alerts=set()
    for edge in EDGES:
        affected=blocked_labels.intersection(edge.get("obstacle_classes",[]))
        if affected: alerts.update(affected)
    if alerts:
        result.insert(-1 if len(result)>1 else 0,{"step":None,"alert":True,"text":f"Obstacle alert: detected {', '.join(sorted(alerts))}. Routes near affected walkways were avoided where possible."})
    return result
