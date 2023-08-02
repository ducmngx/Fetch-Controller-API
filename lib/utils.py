import numpy as np

class Algorithms:
    '''
    Optimization algorithm to dictate joints' behaviors
    '''
    def __init__(self) -> None:
        pass

class MatrixOperation:

    def __init__(self) -> None:
        self.axes = np.array([0, 0, 0, 1])

    def setAxes(self, axes=[0, 0, 0]) -> None:
        self.axes = np.array([axes[0], axes[1], axes[2], 1])

    def resetAxes(self) -> None:
        self.setAxes()

    def getAxes(self, M):
        return M[:3, 3]

    def trig(self, angle, unit='radian'):
        if unit != 'radian':
            r = np.radians(angle)
            return np.cos(r), np.sin(r)
        else: 
            return np.cos(angle), np.sin(angle)
    
    def getTransformMatrix(self, rotation=(0,0,0), translation=(0,0,0)):
        xC, xS = self.trig(rotation[0])
        yC, yS = self.trig(rotation[1])
        zC, zS = self.trig(rotation[2])
        dX = translation[0]
        dY = translation[1]
        dZ = translation[2]
        return np.array([[yC*xC, -zC*xS+zS*yS*xC, zS*xS+zC*yS*xC, dX],
            [yC*xS, zC*xC+zS*yS*xS, -zS*xC+zC*yS*xS, dY],
            [-yS, zS*yC, zC*yC, dZ],
            [0, 0, 0, 1]])
    
    def getInverseMatrix(self, A):
        return np.linalg.inv(A)
    
