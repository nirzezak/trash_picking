import pybullet as p


class Score(object):
    def __init__(self, location=None):
        if not location:
            location = [0, 0, 2]
        self.location = location
        self.score = 0
        self.color = [1, 0, 0]
        msg = f'score = {self.score}'
        self.id = p.addUserDebugText(msg, self.location, textColorRGB=self.color)

    def increase_score(self):
        self.score += 1
        msg = f'score = {self.score}'
        self.id = p.addUserDebugText(msg, self.location, textColorRGB=self.color,
                                     replaceItemUniqueId=self.id)