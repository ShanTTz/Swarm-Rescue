"""Local right of way from fresh radio positions and advertised motion goals."""
import math
from collections import deque


class CarrierRightOfWay:
    def __init__(self):
        self.owner = None
        self.target = None
        self.origin = None
        self.axis = None
        self.started = -1000
        self.cooldown_until = -1
        self.history = {}
        self.carrier_start = None

    @staticmethod
    def direction(point, velocity, heading, goal):
        if goal is not None and math.dist(point, goal) > 15:
            dx, dy = goal[0]-point[0], goal[1]-point[1]
        elif math.hypot(*velocity) > .3:
            dx, dy = velocity
        else:
            dx, dy = math.cos(heading), math.sin(heading)
        length = math.hypot(dx, dy)
        return dx/length, dy/length

    def clear(self):
        self.owner = self.target = self.origin = self.axis = None

    def choose(self, position, own_id, loaded, step, peers, goals, free, own_goal=None, eligible=None):
        if loaded:
            self.clear()
            return None
        fresh = {owner: data for owner, data in peers.items()
                 if owner != own_id and 0 <= step-data[2] <= 6 and data[1] > 0}
        for owner, data in fresh.items():
            trail=self.history.setdefault(owner,deque(maxlen=9))
            if trail and step-trail[-1][0]>8:
                trail.clear()
            if not trail or step-trail[-1][0]>=4:
                trail.append((step,data[0],data[4]))
        if self.owner is not None:
            peer = fresh.get(self.owner)
            age = step-self.started
            if peer is None:
                self.clear()
                return None
            passed = peer is None or not peer[4]
            if peer is not None:
                delta = self.origin[0]-peer[0][0], self.origin[1]-peer[0][1]
                passed |= math.hypot(*delta)>110 or sum(a*b for a,b in zip(delta,self.axis)) < -45
            if passed and age >= 16 or age >= 80:
                self.clear()
                self.cooldown_until = step+12
            elif self.target is not None:
                return self.target
        if step < self.cooldown_until:
            return None
        conflicts = []
        for owner, peer in fresh.items():
            if eligible is not None and owner not in eligible:
                continue
            point, _, _, heading, carrying, velocity = peer
            if not carrying or math.dist(position, point) > 65:
                continue
            trail=self.history[owner]
            if (len(trail)<7 or step-trail[0][0]<24 or not all(p[2] for p in trail)
                    or math.dist(trail[0][1],point)>30
                    or max(math.dist(trail[0][1],p[1]) for p in trail)>55):
                continue
            # A lone crossing remains on the original motion policy. Intervene
            # in a persistent queue, not every slow turn or pickup encounter.
            empty_neighbours=sum(not data[4] and math.dist(data[0],point)<75
                                 for other,data in fresh.items() if other!=owner)
            if empty_neighbours < 1:
                continue
            axis = self.direction(point, velocity, heading, goals.get(owner))
            delta = position[0]-point[0], position[1]-point[1]
            along = delta[0]*axis[0]+delta[1]*axis[1]
            across = abs(delta[0]*axis[1]-delta[1]*axis[0])
            if -20 < along < 65 and across < 38 or math.hypot(*delta) < 35:
                conflicts.append((math.hypot(*delta),owner,peer,axis))
        if not conflicts:
            return None
        _, owner, peer, axis = min(conflicts, key=lambda item:(item[0],item[1]))
        choices=[]
        for length in (55.,80.,40.):
            for index in range(16):
                angle=2*math.pi*index/16
                target=position[0]+length*math.cos(angle),position[1]+length*math.sin(angle)
                delta=target[0]-peer[0][0],target[1]-peer[0][1]
                across=abs(delta[0]*axis[1]-delta[1]*axis[0])
                if across < 48 or math.hypot(*delta) < 55:
                    continue
                separation=min((math.dist(target,p[0]) for p in fresh.values()),default=200)
                if separation < 32 or not free(position,target):
                    continue
                # Compact refuge, away from other vehicles; deterministic tie.
                cost=.15*length+.04*max(0,80-separation)+.005*index
                if own_goal is not None:
                    cost += .25*(math.dist(target,own_goal)-math.dist(position,own_goal))
                choices.append((cost,target))
        if not choices:
            return None
        _,self.target=min(choices)
        self.owner,self.origin,self.axis,self.started=owner,position,axis,step
        self.carrier_start=peer[0]
        return self.target


class EmptyCongestionRecovery:
    """A bounded escape after measured empty-drone collision/stall evidence."""
    def __init__(self):
        self.trail=deque(maxlen=9)
        self.target=None
        self.started=-1000

    def clear(self):
        self.trail.clear()
        self.target=None

    def choose(self, position, own_id, loaded, health, step, peers, wall_near,
               visible_bomb, own_goal, free):
        if loaded:
            self.clear()
            return None
        fresh={owner:data for owner,data in peers.items()
               if owner!=own_id and 0<=step-data[2]<=6 and data[1]>0}
        nearby={owner:data for owner,data in fresh.items() if math.dist(position,data[0])<65}
        if self.target is not None:
            if visible_bomb and not any(math.dist(position,data[0])<34 for data in nearby.values()):
                self.clear()
                return None
            if step-self.started<80:
                return self.target
            self.clear()
            return None
        if not self.trail or step-self.trail[-1][0]>=8:
            self.trail.append((step,position,health))
        if len(self.trail)<9:
            return None
        first,origin,old_health=self.trail[0]
        if (step-first<64 or math.dist(origin,position)>30
                or max(math.dist(origin,p) for _,p,_ in self.trail)>50):
            return None
        # Geometry drives recovery, not the engine's wall-clock HP cooldown.
        if not wall_near and not any(math.dist(position,data[0])<36 for data in nearby.values()):
            return None
        # Keep one stable owner moving; lower-priority peers clear the queue.
        if not wall_near and not any(data[4] or owner<own_id for owner,data in nearby.items()):
            return None
        if visible_bomb and not any(math.dist(position,data[0])<34 for data in nearby.values()):
            return None
        choices=[]
        for length in (45.,65.,90.):
            for index in range(16):
                angle=2*math.pi*index/16
                target=position[0]+length*math.cos(angle),position[1]+length*math.sin(angle)
                separation=min((math.dist(target,p[0]) for p in fresh.values()),default=100.)
                if separation<40 or not free(position,target):
                    continue
                progress=(math.dist(target,own_goal)-math.dist(position,own_goal)) if own_goal is not None else 0
                cost=.15*length-.5*min(100,separation)+.08*progress+.003*((index-own_id)%16)
                choices.append((cost,target))
        if not choices:
            return None
        _,self.target=min(choices)
        self.started=step
        return self.target
