import numpy as np

class Config:
    def __init__(self, H, W, G, k, wall_thickness, tolerance):
        self.G = G
        self.H = H 
        self.W = W
        self.k = k
        self.t = wall_thickness
        self.tol = tolerance

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y 
    
    def __mul__(self, v):
        self.x *= v.x
        self.y *= v.y
        return self
    
    def __add__(self, v):
        self.x = self.x + v.x 
        self.y = self.y + v.y 
        return self

    def __sub__(self, v):
        self.x = self.x - v.x 
        self.y = self.y - v.y 
        return self

    def __and__(self, v): ## return cosine of angle
        return (v.x - self.x)/(np.sqrt((v.x - self.x)**2 + (v.y - self.y)**2))

    def __or__(self, v): ## return sine of angle
        return (v.y - self.y)/(np.sqrt((v.x - self.x)**2 + (v.y - self.y)**2))

class Body:
    def __init__(self, name, mass, radius, pose, vel, config):
        self.name = name
        self.m = mass
        self.r = radius
        self.pose = Vector(pose[0], pose[1])
        self.vel  = Vector(vel[0], vel[1])
        self.acc  = Vector(0., 0.)
        self.config = config

    def move(self, dpose):
        self.pose = self.pose + dpose 
        self.pose = Vector(max(-self.config.tol, min(self.config.W+self.config.tol, self.pose.x)), max(-self.config.tol, min(self.config.H+self.config.tol, self.pose.y)))

    def accelerate(self, dvel):
        self.vel = self.vel + dvel 
        self.vel.x = max(-5, min(5, self.vel.x))
        self.vel.y = max(-5, min(5, self.vel.y))

    def pose_change(self, T):
        d = Vector(0, 0)
        d.x = self.vel.x*T + 0.5*self.acc.x*T**2
        d.y = self.vel.y*T + 0.5*self.acc.y*T**2
        return d 

    def vel_change(self, T):
        d = Vector(0, 0)
        d.x = self.acc.x*T
        d.y = self.acc.y*T
        return d        

    def __eq__(self, body):
        if body.name == self.name:
            return True 
        else:
            return False

    def update_acc(self, body):
            r_sq = (self.pose.x - body.pose.x)**2 + (self.pose.y - body.pose.y)**2 + 1
            self.dist = np.sqrt(r_sq)

            force_unitmass = self.config.G*body.m*(1/r_sq)
            ax = force_unitmass* (self.pose & body.pose) ## cosine of angle
            ay = force_unitmass* (self.pose | body.pose) ## sine of angle
            
            self.acc = self.acc + Vector(ax, ay)

            if collision(self, body):
                deformation = Deformation(self, body)
                force_unitmass = self.config.k*deformation*(1/self.m)
                ax = -1 * force_unitmass * (self.pose & body.pose) ## cosine of angle
                ay = -1 * force_unitmass * (self.pose | body.pose) ## sine of angle
                self.acc = self.acc + Vector(ax, ay)
    
    def wall_collision(self):
        deformation = 0
        ax, ay = 0, 0
        if self.pose.x>self.config.W:
            deformation = self.pose.x - self.config.W
            ax = -1
        elif self.pose.y>self.config.H:
            deformation = self.pose.y - self.config.H
            ay = -1
        elif self.pose.x<0:
            deformation = 0 - self.pose.x
            ax = 1
        elif self.pose.y<0:
            deformation = 0 - self.pose.y
            ay = 1
        else:
            return
        force_unitmass = self.config.k*deformation*(1/self.m)
        ax *= force_unitmass ## cosine of angle
        ay *= force_unitmass ## sine of angle
        self.acc = self.acc + Vector(ax, ay)

def Deformation(A, B):
    dist = np.sqrt((A.pose.x - B.pose.x)**2 + (A.pose.y - B.pose.y)**2)
    min_dist = (A.r + B.r)
    return min_dist-dist

def collision(A, B) -> bool:
    dist_squared = (A.pose.x - B.pose.x)**2 + (A.pose.y - B.pose.y)**2
    min_dist_squared = (A.r + B.r)**2
    if dist_squared-min_dist_squared<0:
        return True 
    else:
        return False
