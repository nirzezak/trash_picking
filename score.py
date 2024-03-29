from typing import Optional, List

import pybullet as p


class Score(object):
    def __init__(self, prefix: str, color: Optional[List[float]] = None, location: Optional[List[int]] = None):
        """
        :param prefix: prefix to the score message. Final message will be '{prefix} = <score>'
        :param color: list of RGB values to use as the color of the score
        :param location: location to spawn the score (in pybullet)
        """
        if not location:
            location = [0, 0, 2]
        if not color:
            color = [1, 0, 0]
        self.location = location
        self.score = 0
        self.color = color
        self.prefix = prefix
        msg = f'{self.prefix} = {self.score}'
        self.id = p.addUserDebugText(msg, self.location, textColorRGB=self.color)

    def increase_score(self, amount=1):
        """
        Increase the score

        :param amount: how much to increase the score. Default is 1
        """
        if amount == 0:
            return
        p.removeUserDebugItem(self.id)
        self.score += amount
        msg = f'{self.prefix} = {self.score}'
        self.id = p.addUserDebugText(msg, self.location, textColorRGB=self.color,
                                     replaceItemUniqueId=self.id)
