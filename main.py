#https://www.google.com/search?q=google+colour+picker&sca_esv=47a159505e5d460b&sca_upv=1&ei=3jTnZfH0JKjy4-EPz9uN2A4&ved=0ahUKEwjx2-bzst2EAxUo-TgGHc9tA-sQ4dUDCBA&uact=5&oq=google+colour+picker&gs_lp=Egxnd3Mtd2l6LXNlcnAiFGdvb2dsZSBjb2xvdXIgcGlja2VyMggQABiABBixAzIFEAAYgAQyBRAAGIAEMgUQABiABDIFEAAYgAQyBRAAGIAEMgUQABiABDIFEAAYgAQyBhAAGBYYHjIGEAAYFhgeSOwaUJgFWNgZcAN4AZABAJgBxgGgAfwIqgEDMC43uAEDyAEA-AEBmAIKoALZCcICChAAGEcY1gQYsAPCAgsQABiABBiKBRiRAsICBxAAGIAEGArCAgsQABiABBiKBRiGA8ICChAAGIAEGAoYsQOYAwCIBgGQBgiSBwUzLjYuMaAH6Ck&sclient=gws-wiz-serp
#52, 137, 235
#Source for equations -> https://en.wikipedia.org/wiki/Inelastic_collision
import pygame
import random
import time
from os import system
FoNt = 0
FoNtprint = 0
screenRes = [1200, 800]
class particle:
    neighbours = [[-1, -1], [-1, 0], [-1, 1],
                 [0, -1], [0, 0], [0, 1],
                 [1, -1], [1, 0], [1, 1]]
    gfactor = 100   #grid size factor
    gLen = [int(screenRes[0]//gfactor), int(screenRes[1]//gfactor)]
    grid = list()
    radius = 10
    mass = 10
    isSurfaceSmooth = True
    friction = 25*4
    minimumDistanceOfParticles = 0.5
    e = 0.9 #coefficient of restitution
    maxVelocity = 500
    gravity = 981*2
    attractionConstant = 145889899*1.2*radius/2
    distanceOfZeroInfluence = radius * 10
    wallVelocityReduction = 0.95
    nearParticleVelReduction = 0.999
    rate = 0.5
    surfaceBoundary = [50, 50]
    collisions = list()
    killMomentum = False
    
    for i in range(gLen[0]):
        grid.append(list())
        for j in range(gLen[1]):
            grid[i].append(list())
    def __init__(self, pos, vel) -> None:
        self.pos = pygame.math.Vector2(pos)
        self.vel = pygame.math.Vector2(vel)
        self.acc = pygame.math.Vector2(0)
        self.isNearWall = False
        self.color = pygame.Color((52, 137, 235)) 
        self.nearParticleCount = 0
        self.nearParticleSumDirection = pygame.math.Vector2(0)
        self.gi = [int(pos[0]//particle.gfactor), int(pos[1]//particle.gfactor)]
        particle.grid[self.gi[0]][self.gi[1]].append(self)
    def update(self, dt):
        self.vel += self.acc*dt*particle.rate
        self.vel[1] += particle.gravity*dt*particle.rate
        velMagnitude = self.vel.magnitude()
        ratio = velMagnitude/particle.maxVelocity
        if ratio > 1:
            self.vel /= ratio
        if particle.isSurfaceSmooth == False:
            if particle.friction*dt < velMagnitude:
                self.acc = pygame.math.Vector2(particle.friction)
                self.acc = self.acc.rotate(self.acc.angle_to(self.vel))
                self.vel -= self.acc*dt*particle.rate
            else:
                self.vel = pygame.math.Vector2(0)
        self.acc = pygame.math.Vector2(0)
        if particle.killMomentum == True:
            self.vel = pygame.math.Vector2(0)
        if self.nearParticleSumDirection.magnitude() > 0.1:
            # self.vel *= particle.nearParticleVelReduction**(self.nearParticleCount/2)
            velComponent = self.vel.project(self.nearParticleSumDirection)
            velOtherComponent = self.vel-velComponent
            velComponent *= particle.nearParticleVelReduction**(self.nearParticleCount/1)
            velOtherComponent *= particle.nearParticleVelReduction**(self.nearParticleCount/2)
            self.vel = velOtherComponent + velComponent
        self.pos += self.vel*dt*particle.rate
        self.nearParticleCount = 0
        self.isNearWall = False
        if self.pos[0] < particle.surfaceBoundary[0]:
            self.pos[0] = particle.surfaceBoundary[0]+1
            self.vel[0] = -self.vel[0]*particle.wallVelocityReduction
            self.isNearWall = True
        elif self.pos[0] > screenRes[0]-particle.surfaceBoundary[0]:
            self.pos[0] = screenRes[0]-particle.surfaceBoundary[0]-1
            self.vel[0] = -self.vel[0]*particle.wallVelocityReduction
            self.isNearWall = True
        
        if self.pos[1] < particle.surfaceBoundary[1]:
            self.pos[1] = particle.surfaceBoundary[1]+1
            self.vel[1] = -self.vel[1]*particle.wallVelocityReduction
            self.isNearWall = True
        elif self.pos[1] > screenRes[1]-particle.surfaceBoundary[1]:
            self.pos[1] = screenRes[1]-particle.surfaceBoundary[1]-1
            self.vel[1] = -self.vel[1]*particle.wallVelocityReduction
            self.isNearWall = True

        self.color = pygame.Color(veltoColor(self.vel.magnitude()/particle.maxVelocity))

        particle.grid[self.gi[0]][self.gi[1]].remove(self)
        self.gi = [int(self.pos[0]//particle.gfactor), int(self.pos[1]//particle.gfactor)]
        particle.grid[self.gi[0]][self.gi[1]].append(self)

    def collision(self, dt) -> None:
        for i in particle.neighbours:
            x = self.gi[0] + i[0]
            y = self.gi[1] + i[1]
            if 0<=x<particle.gLen[0] and 0<=y<particle.gLen[1]:
                for elem in particle.grid[x][y]:
                    if elem != self:
                        dist = (elem.pos-self.pos).magnitude()
                        newA= pygame.math.Vector2(((particle.attractionConstant)/((dist*dist)**1.75 + particle.minimumDistanceOfParticles)))
                        newA = newA.rotate(newA.angle_to(elem.pos-self.pos))
                        self.acc -= newA
                        self.nearParticleCount += (1+self.isNearWall*15)*(particle.gfactor*1.5)/(dist+1)
                        if elemVelMag:=elem.vel.magnitude() > 0.1:
                            self.nearParticleSumDirection += (particle.gfactor*1.5/(dist+1))*elem.vel/elemVelMag
                        if (elem, self) not in particle.collisions:
                            if dist < particle.radius*2:
                                direction = self.pos-elem.pos
                                if direction.magnitude() > 0.1:
                                    u1 = self.vel.project(direction)
                                    u2 = elem.vel.project(direction)
                                    v1 = (particle.e*(u2-u1) + u1 + u2)/2
                                    v2 = (particle.e*(u1-u2) + u1 + u2)/2
                                    self.vel = self.vel-u1+v1
                                    elem.vel = elem.vel-u2+v2
                                    particle.collisions.append((self, elem))
        if isMousePressed == True:
            dist = (mpos-self.pos).magnitude()
            newA= 90*pygame.math.Vector2(((particle.attractionConstant)/((dist*dist)**1.5 + particle.minimumDistanceOfParticles)))
            newA = newA.rotate(newA.angle_to(mpos-self.pos))
            self.acc -= newA
                        
    
    def draw(self):
        pygame.draw.circle(screen, pygame.Color(self.color), self.pos, particle.radius)

def veltoColor(ratio):
    cRatio = 647*ratio
    COLOR = [52, 137, 235]
    if cRatio <= 98:
        COLOR = [52, 137 + cRatio, 235]
    elif cRatio <= 281:
        COLOR = [52, 235, 235+98 - cRatio]
    elif cRatio <= 464:
        COLOR = [52+cRatio-281, 235, 52]
    else:
        COLOR = [235, 235+464 - cRatio, 52]
    return COLOR
def cls():
    system("cls")
def font(face:str,size=18):
    global FoNt
    FoNt = pygame.font.SysFont(face,size)
def printpy(text:str,coords=(100,400),color=(128,128,128)):
    global FoNt,FoNtprint
    FoNtprint = FoNt.render(text,True,color)
    screen.blit(FoNtprint,coords)
if __name__ == "__main__":
    frameRate = 1000
    dt = 1/1000
    pygame.init()
    screen = pygame.display.set_mode(screenRes)
    #icon = pygame.image.load('')
    pygame.display.set_caption("")
    #pygame.display.set_icon(icon)
    isMousePressed = False
    cls()
    obj = list()
    for i in range(240):
        obj.append(particle([particle.surfaceBoundary[0]+random.random()*(screenRes[0]-2*particle.surfaceBoundary[0]), 
                             particle.surfaceBoundary[1]+random.random()*(screenRes[1]-2*particle.surfaceBoundary[1])],

                            [random.random()*particle.maxVelocity,
                             random.random()*particle.maxVelocity]))
        
    running = True
    clock = pygame.time.Clock()
    while running == True:
        initTime = time.time()
        clock.tick(frameRate*1.769)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                # particle.killMomentum = True
                # particle.maxVelocity = 50 if particle.maxVelocity == 300 else 300
                # print(frameRate, "  <+++>  ", particle.maxVelocity)
                isMousePressed = True
            if event.type == pygame.MOUSEBUTTONUP:
                isMousePressed = False
        particle.collisions = list()
        if isMousePressed == True:
            mpos = pygame.math.Vector2(pygame.mouse.get_pos())

        #Code Here
        if particle.killMomentum == True:
            particle.killMomentum = False
        screen.fill((128, 128, 128))
        for elem in obj:
            elem.update(dt)
            elem.draw()
            elem.collision(dt)
        
        pygame.display.update()
        endTime = time.time()
        dt = endTime-initTime
        if dt != 0:
            frameRate = 1/dt
        else:
            frameRate = 1000
