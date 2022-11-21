import numpy as np
import cv2
import time

from utils import Body, Vector, Config, collision

T = 0.5

config = Config(700, 700, 1000, 5, 5, 5)

def get_color(i):
    b = abs(int(3*i+45))/256
    g = abs(int(27*i+145))/256
    r = abs(int(25*i+45/(i+1)+105))/256
    return (b, g, r)

def plot(image, body):
    print(body.pose.x, body.pose.y)
    return cv2.circle(image, (int(body.pose.x), int(body.pose.y)), int(body.r), get_color(body.name), -1)

def create_platform(H, W):
    image = np.zeros((config.W, config.H, 3))
    image[0:config.t,:,:] = 255
    image[-config.t:,:,:] = 255
    image[:,0:config.t,:] = 255
    image[:,-config.t:,:] = 255
    return image

def move(bodies):
    image = create_platform(config.W, config.H)
    while True:
        image = create_platform(config.W, config.H)
        for body1 in bodies:
            body1.acc = Vector(0., 0.)
            for body2 in bodies:
                if body1 == body2:
                    continue
                body1.update_acc(body2)
            
            body1.wall_collision()
            
            dpose = body1.pose_change(T)
            body1.move(dpose)

            dvel  = body1.vel_change(T)
            body1.accelerate(dvel)
            
            image = plot(image, body1)
        cv2.imshow("Output", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            return False
        elif key == ord('r'):
            cv2.destroyAllWindows()
            return True

if __name__=="__main__":
    num_body = 4
    mass = [1,1,1,1]
    radius = [5, 5, 20]
    bodies = []
    repeat = True
    while(repeat):
        for i in range(num_body):
            bodies.append(Body(i, mass[i], 5, np.random.randint(config.tol, config.W-config.tol, 2), np.random.randint(0, 1, 2), config))
            bodies[-1].vel = Vector(0., 0.)
        repeat = move(bodies)

