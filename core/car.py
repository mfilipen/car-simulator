import core.state as st
import numpy as np

class Car(object):
    def __init__(self, state=None):

        if state is None:
            self.state = st.State(0.65, 0.35, np.pi / 2, 0)

        else:
            self.state = state

    def get_state(self):
        return self.state