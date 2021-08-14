from message.vision import Goal

from nuclear import Reactor, Trigger, on


@Reactor
class PyModule(object):
    def __init__(self) -> None:
        super().__init__()

        self.goal = Goal()

    @on(Trigger(Goal))
    def test_method(self, goal):
        print("Nice")
