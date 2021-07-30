from message.vision import Goal

# from nuclear import Every, Reactor, Single, Trigger, With, on
import nuclear


@Reactor
class PyModule(object):
    def __init__(self) -> None:
        super().__init__()

        self.goal = Goal()

    @on(Trigger(Goal))
    def test_method(self, goal):
        print("Nice")
