class AutomatedCommand:
    def __init__(self, dX: float=0, dY: float = 0, dTheta: float = 0):
        self.dX = dX
        self.dY = dY
        self.dTheta = dTheta
        
    def get(self):
        return self.dX, self.dY, self.dTheta
        
    def set(self, dX: int, dY: int, dTheta: int):
        self.dX = dX
        self.dY = dY
        self.dTheta = dTheta
    